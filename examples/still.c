/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/capture.h>
#include <ccamera/configuration.h>
#include <dirent.h>
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <linux/dma-buf.h>
#include <linux/media-bus-format.h>
#include <linux/videodev2.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include <time.h>
#include <unistd.h>

struct row {
    uint32_t sequence, flags;
    uint64_t timestamp;
    cc_ipa_metadata ipa;
    cc_sensor_controls sensor;
    double luma;
};
static uint64_t now_ns(void)
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t))
        return 0;
    return (uint64_t)t.tv_sec * 1000000000U + (uint64_t)t.tv_nsec;
}
static int number(const char *s, uint32_t *n)
{
    char *end;
    errno = 0;
    unsigned long v = strtoul(s, &end, 10);
    if (errno || !*s || *s == '-' || *end || v > UINT32_MAX)
        return -EINVAL;
    *n = (uint32_t)v;
    return 0;
}
static int write_payload(FILE *out, const cc_frame *frame)
{
    for (uint32_t i = 0; i < frame->num_planes; ++i) {
        const cc_plane *p = &frame->planes[i];
        if (!p->data || p->data_offset > p->bytes_used || p->bytes_used > p->length)
            return -EPROTO;
        size_t n = p->bytes_used - p->data_offset;
        if (fwrite((const unsigned char *)p->data + p->data_offset, 1, n, out) != n)
            return -EIO;
    }
    return 0;
}
static int ppm(const char *path, const cc_frame *frame, const cc_format *format)
{
    if (frame->num_planes != 1 || format->fourcc != V4L2_PIX_FMT_RGB24)
        return -ENOTSUP;
    const cc_plane *p = &frame->planes[0];
    uint64_t last = (uint64_t)(format->height - 1) * format->planes[0].stride + (uint64_t)format->width * 3;
    if (!p->data || p->data_offset > p->bytes_used || p->bytes_used > p->length ||
        last > p->bytes_used - p->data_offset)
        return -EPROTO;
    FILE *out = fopen(path, "wb");
    if (!out)
        return -errno;
    int ret = fprintf(out, "P6\n%u %u\n255\n", format->width, format->height) < 0 ? -EIO : 0;
    for (uint32_t y = 0; !ret && y < format->height; ++y) {
        const unsigned char *bytes =
            (const unsigned char *)p->data + p->data_offset + (size_t)y * format->planes[0].stride;
        size_t n = (size_t)format->width * 3;
        if (fwrite(bytes, 1, n, out) != n)
            ret = -EIO;
    }
    if (fclose(out) && !ret)
        ret = -EIO;
    return ret;
}
static int disable_auto(cc_camera *camera)
{
    const uint32_t ids[] = {V4L2_CID_AUTOGAIN, V4L2_CID_AUTO_WHITE_BALANCE, V4L2_CID_EXPOSURE_AUTO};
    for (unsigned i = 0; i < 3; ++i) {
        cc_control_info info;
        int ret = cc_camera_query_control(camera, ids[i], &info);
        if (ret == -EINVAL || ret == -ENOENT || ret == -ENOTTY)
            continue;
        if (ret < 0)
            return ret;
        cc_control c = {.id = ids[i], .value = i == 2 ? V4L2_EXPOSURE_MANUAL : 0};
        ret = cc_camera_set_controls(camera, &c, 1, NULL);
        if (ret < 0)
            return ret;
    }
    return 0;
}
static int metadata(const char *path, const struct row *rows, size_t n, const cc_sensor_trace *trace,
                    size_t traces, uint64_t dropped, const cc_format *raw, const cc_format *rgb,
                    const cc_sensor_geometry *geometry, const uint32_t delays[4], const char *diagnostic,
                    int status, const cc_sensor_timing *timing, enum cc_orientation orientation,
                    const cc_vc4_profile_info *profile)
{
    if (!path)
        return 0;
    FILE *out = fopen(path, "w");
    if (!out)
        return -errno;
    struct utsname kernel = {0};
    (void)uname(&kernel);
    fprintf(out,
            "{\"type\":\"platform\",\"kernel\":\"%s\",\"machine\":\"%s\",\"pixel_rate\":%" PRIu64
            ",\"hblank\":%u,\"vblank\":%u}\n",
            kernel.release, kernel.machine, timing->pixel_rate, timing->hblank, timing->vblank);
    fprintf(out,
            "{\"type\":\"configuration\",\"profile\":\"%s\",\"diagnostic\":\"%s\",\"status\":%d,\"raw_"
            "fourcc\":%u,\"width\":%u,\"height\":%u,\"raw_stride\":%u,\"rgb_stride\":%u,\"crop\":[%d,%d,%u,%"
            "u],\"delays\":[%u,%u,%u,%u],\"trace_dropped\":%" PRIu64
            ",\"orientation\":%u,\"hflip\":%u,\"vflip\":%u}\n",
            profile ? profile->calibration_id : "", diagnostic ? diagnostic : "", status, raw->fourcc, raw->width,
            raw->height, raw->planes[0].stride, rgb->planes[0].stride, geometry->crop.x, geometry->crop.y,
            geometry->crop.width, geometry->crop.height, delays[0], delays[1], delays[2], delays[3], dropped,
            (unsigned)orientation, geometry->hflip, geometry->vflip);
    for (size_t i = 0; i < n; ++i) {
        const struct row *r = &rows[i];
        const cc_ipa_metadata *m = &r->ipa;
        fprintf(out,
                "{\"type\":\"frame\",\"sequence\":%u,\"timestamp_ns\":%" PRIu64
                ",\"flags\":%u,\"luminance\":%.9g,\"exposure_ns\":%" PRIu64
                ",\"gain\":%.9g,\"red_gain\":%.9g,\"blue_gain\":%.9g,\"colour_temperature\":%.9g,\"frame_"
                "duration_ns\":%" PRIu64 ",\"controls\":[",
                r->sequence, r->timestamp, m->flags, r->luma, m->exposure_ns, m->analogue_gain, m->red_gain,
                m->blue_gain, m->colour_temperature, m->frame_duration_ns);
        for (uint32_t j = 0; j < r->sensor.count; ++j)
            fprintf(out, "%s{\"id\":%u,\"expected\":%" PRId64 "}", j ? "," : "", r->sensor.controls[j].id,
                    r->sensor.controls[j].value);
        fputs("]}\n", out);
    }
    for (size_t i = 0; i < traces; ++i) {
        const cc_sensor_trace *t = &trace[i];
        fprintf(out,
                "{\"type\":\"write\",\"sequence\":%u,\"target\":%u,\"id\":%u,\"sof_ns\":%" PRIu64
                ",\"start_ns\":%" PRIu64 ",\"end_ns\":%" PRIu64 ",\"requested\":%" PRId64
                ",\"acknowledged\":%" PRId64 ",\"error\":%d}\n",
                t->frame_sequence, t->target_sequence, t->id, t->sof_ns, t->write_start_ns, t->write_end_ns,
                t->requested, t->acknowledged, t->error);
    }
    int ret = ferror(out) ? -EIO : 0;
    if (fclose(out) && !ret)
        ret = -EIO;
    return ret;
}
static int open_fds(void)
{
    DIR *dir = opendir("/proc/self/fd");
    if (!dir)
        return -errno;
    int count = 0;
    struct dirent *entry;
    errno = 0;
    while ((entry = readdir(dir)))
        if (entry->d_name[0] != '.')
            ++count;
    int ret = errno ? -errno : count;
    if (closedir(dir) && ret >= 0)
        ret = -errno;
    return ret;
}
static uint64_t sample(const unsigned char *data, size_t size)
{
    uint64_t hash = UINT64_C(14695981039346656037);
    for (size_t i = 0; i < size; i += 1021)
        hash = (hash ^ data[i]) * UINT64_C(1099511628211);
    return (hash ^ data[size - 1]) * UINT64_C(1099511628211);
}
static int sync_read(int fd, uint64_t phase)
{
    struct dma_buf_sync sync = {.flags = phase | DMA_BUF_SYNC_READ};
    int ret;
    do {
        ret = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    } while (ret < 0 && (errno == EINTR || errno == EAGAIN));
    return ret < 0 ? -errno : 0;
}
static int check_export(cc_capture *capture, const cc_request *request, enum cc_capture_stream stream,
                        const cc_plane *plane, uint64_t expected)
{
    int fd = cc_capture_export_plane(capture, request, stream, 0);
    if (fd < 0)
        return fd;
    const unsigned char *map = mmap(NULL, plane->length, PROT_READ, MAP_SHARED, fd, 0);
    int ret = map == MAP_FAILED ? -errno : sync_read(fd, DMA_BUF_SYNC_START);
    if (!ret) {
        uint64_t hash = sample(map + plane->data_offset, plane->bytes_used - plane->data_offset);
        ret = sync_read(fd, DMA_BUF_SYNC_END);
        if (!ret && hash != expected)
            ret = -EIO;
    }
    if (map != MAP_FAILED && munmap((void *)map, plane->length) && !ret)
        ret = -errno;
    if (close(fd) && !ret)
        ret = -errno;
    return ret;
}
static int repeat_result(cc_capture *capture, cc_request *request, uint64_t sequence, uint64_t *timestamp,
                         bool export_images, uint64_t hashes[2], FILE *log)
{
    const cc_capture_result *r = cc_request_get_result(request);
    if (r && r->error)
        return r->error;
    if (!r || r->status != CC_REQUEST_COMPLETE || r->sequence != sequence || r->cookie != sequence + 1 ||
        r->raw.timestamp_ns <= *timestamp || !r->sensor.count || r->sensor.frame_sequence != r->raw.sequence)
        return -EPROTO;
    const cc_frame *images[] = {&r->raw, &r->output[0]};
    const enum cc_capture_stream streams[] = {CC_CAPTURE_RAW, CC_CAPTURE_OUTPUT0};
    for (size_t i = 0; i < 2; ++i) {
        const cc_plane *p = &images[i]->planes[0];
        if (images[i]->num_planes != 1 || !p->data || p->data_offset >= p->bytes_used || p->bytes_used > p->length)
            return -EPROTO;
        hashes[i] = sample((const unsigned char *)p->data + p->data_offset, p->bytes_used - p->data_offset);
        if (export_images) {
            int ret = check_export(capture, request, streams[i], p, hashes[i]);
            if (ret < 0)
                return ret;
        }
    }
    *timestamp = r->raw.timestamp_ns;
    if (!log)
        return 0;
    return fprintf(log, "{\"type\":\"frame\",\"request\":%" PRIu64 ",\"cookie\":%" PRIu64 ",\"sensor_sequence\":%u,"
                        "\"sensor_cookie\":%" PRIu64 ",\"exposure_ns\":%" PRIu64 ",\"analogue_gain\":%.6g,"
                        "\"raw_ns\":%" PRIu64 ",\"rgb_ns\":%" PRIu64 ",\"raw_sample\":\"%016" PRIx64
                        "\",\"rgb_sample\":\"%016" PRIx64 "\",\"dma_checked\":%s}\n",
                   sequence, r->cookie, r->raw.sequence, r->sensor.cookie, r->ipa.exposure_ns, r->ipa.analogue_gain,
                   *timestamp, r->output[0].timestamp_ns, hashes[0], hashes[1],
                   export_images ? "true" : "false") < 0 ? -EIO : 0;
}
static int repeat_check(cc_capture **graph, uint32_t slots, uint32_t frames, int timeout,
                        const cc_control *manual, size_t num_manual, FILE *log)
{
    cc_request *requests[256] = {0}, *completed = NULL;
    if (!slots || slots >= sizeof(requests) / sizeof(requests[0]))
        return -ENOSPC;
    cc_capture *capture = *graph;
    uint64_t timestamp = 0, held[2] = {0}, hashes[2];
    int ret = 0;
    const char *step = "holding all capture slots";
    for (uint32_t i = 0; i <= slots; ++i) {
        ret = cc_request_create(i + 1, &requests[i]);
        if (!ret)
            ret = cc_request_set_sensor_controls(requests[i], manual, num_manual);
        if (!ret)
            ret = cc_capture_queue(capture, requests[i]);
        if (ret < 0)
            goto end;
    }
    if (cc_capture_queue(capture, requests[0]) != -EBUSY) {
        ret = -EPROTO;
        goto end;
    }
    ret = cc_capture_dequeue(capture, &completed, 1);
    bool timed_out = ret == -ETIMEDOUT;
    if (timed_out && (completed || cc_request_get_status(requests[0]) != CC_REQUEST_PENDING)) {
        ret = -EPROTO;
        goto end;
    }
    if (ret < 0 && !timed_out)
        goto end;
    for (uint32_t i = 0; i < slots; ++i) {
        if (i || timed_out)
            ret = cc_capture_dequeue(capture, &completed, timeout);
        if (ret < 0 || completed != requests[i]) {
            if (!ret)
                ret = -EPROTO;
            goto end;
        }
        ret = repeat_result(capture, completed, i, &timestamp, !i, i ? hashes : held, log);
        if (ret < 0)
            goto end;
    }
    if (cc_capture_dequeue(capture, &completed, timeout) != -EAGAIN || completed != requests[slots - 1] ||
        cc_request_get_status(requests[slots]) != CC_REQUEST_PENDING) {
        ret = -EPROTO;
        goto end;
    }
    const cc_capture_result *first = cc_request_get_result(requests[0]);
    uint64_t before = 0;
    ret = repeat_result(capture, requests[0], 0, &before, false, hashes, NULL);
    if (ret < 0 || memcmp(held, hashes, sizeof(held))) {
        if (!ret)
            ret = -EIO;
        goto end;
    }
    step = "stopping with outstanding requests";
    ret = cc_capture_stop(capture);
    if (ret < 0)
        goto end;
    ret = cc_capture_dequeue(capture, &completed, 0);
    if (ret < 0 || completed != requests[slots] || first->raw.num_planes || first->output[0].num_planes) {
        if (!ret)
            ret = -EPROTO;
        goto end;
    }
    for (uint32_t i = 0; i <= slots; ++i) {
        const cc_capture_result *r = cc_request_get_result(requests[i]);
        if (!r || r->status != CC_REQUEST_CANCELLED || r->error != -ECANCELED) {
            ret = -EPROTO;
            goto end;
        }
        ret = cc_capture_release(capture, requests[i]);
        if (ret < 0)
            goto end;
    }
    if (fprintf(log, "{\"type\":\"held_and_stopped\",\"slots\":%u,\"short_wait_timed_out\":%s}\n",
                slots, timed_out ? "true" : "false") < 0) {
        ret = -EIO;
        goto end;
    }
    step = "reusing the request after stop";
    int steady_fds = open_fds();
    if (steady_fds < 0) {
        ret = steady_fds;
        goto end;
    }
    for (uint32_t i = 0; i < frames; ++i) {
        uint64_t sequence = (uint64_t)slots + 1 + i;
        ret = cc_request_set_cookie(requests[0], sequence + 1);
        if (!ret)
            ret = cc_capture_queue(capture, requests[0]);
        if (!ret)
            ret = cc_capture_dequeue(capture, &completed, timeout);
        if (!ret)
            ret = completed == requests[0] ?
                repeat_result(capture, completed, sequence, &timestamp, !(i % 32), hashes, log) : -EPROTO;
        if (!ret)
            ret = cc_capture_release(capture, completed);
        if (ret < 0)
            goto end;
        if (i % 32 == 31 || i + 1 == frames) {
            ret = cc_capture_stop(capture);
            if (ret < 0)
                goto end;
            int fds = open_fds();
            if (fprintf(log, "{\"type\":\"restart\",\"sustained_frames\":%u,\"open_fds\":%d}\n", i + 1, fds) < 0)
                ret = -EIO;
            else if (fds != steady_fds)
                ret = fds < 0 ? fds : -EIO;
            if (ret < 0)
                goto end;
        }
    }
end:
    if (ret < 0)
        fprintf(stderr, "Repeat check failed while %s: %s\n", step, strerror(-ret));
    cc_capture_close(capture);
    *graph = NULL;
    for (uint32_t i = 0; i <= slots; ++i)
        cc_request_destroy(requests[i]);
    return ret;
}
static void usage(void)
{
    puts("ccamera-still [-c INDEX | -I ID] [-o photo.ppm] [--metadata run.jsonl] [--raw frame.raw]\n"
         "  [--statistics frame.bsta] [--flicker off|50|60] [--timeout-ms 10000]\n"
         "  [--shutter-us N] [--gain-code N] [--colour-gains R,B] [--profile NAME]\n"
         "  [--size WIDTHxHEIGHT] [--repeat-check COUNT --metadata repeat.jsonl]\n"
         "  [--list-profiles]\n"
         "  [--exposure-delay N] [--gain-delay N] [--vblank-delay N] [--hblank-delay N]\n"
         "  [--diagnostic exposure|gain|vblank --frames 96]\n"
         "OV5647, IMX219, IMX477 and IMX708 on Pi 4/earlier. Uses full-resolution mode.\n"
         "Defaults to standard colour; select NoIR/wide modules explicitly.\n"
         "Only Pi 4/OV5647 has a physical acceptance target; validation is pending.\n"
         "Diagnostics require OV5647, fixed illumination and settings.\n"
         "Run ccamera-capture -l to choose a discovery index or camera ID.");
}
int main(int argc, char **argv)
{
    enum { FLICKER = 256, TIMEOUT, SHUTTER, GAIN, WB, ED, GD, VD, HD, META, RAW, STATS, DIAG, FRAMES,
           PROFILE, LIST_PROFILES, SIZE, REPEAT, HELP };
    const struct option opts[] = {{"flicker", 1, 0, FLICKER},
                                  {"camera-id", 1, 0, 'I'},
                                  {"timeout-ms", 1, 0, TIMEOUT},
                                  {"shutter-us", 1, 0, SHUTTER},
                                  {"gain-code", 1, 0, GAIN},
                                  {"colour-gains", 1, 0, WB},
                                  {"exposure-delay", 1, 0, ED},
                                  {"gain-delay", 1, 0, GD},
                                  {"vblank-delay", 1, 0, VD},
                                  {"hblank-delay", 1, 0, HD},
                                  {"metadata", 1, 0, META},
                                  {"raw", 1, 0, RAW},
                                  {"statistics", 1, 0, STATS},
                                  {"diagnostic", 1, 0, DIAG},
                                  {"frames", 1, 0, FRAMES},
                                  {"profile", 1, 0, PROFILE},
                                  {"size", 1, 0, SIZE},
                                  {"repeat-check", 1, 0, REPEAT},
                                  {"list-profiles", 0, 0, LIST_PROFILES},
                                  {"help", 0, 0, HELP},
                                  {0, 0, 0, 0}};
    uint32_t index = 0, timeout = 10000, shutter = 0, gain = 0, delays[4] = {2, 2, 2, 2}, frames = 96;
    uint32_t output_width = 0, output_height = 0, repeat = 0;
    cc_ipa_vc4_options tuning = {.mains_hz = 50};
    bool delay_override[4] = {false};
    bool index_selected=false, gain_selected=false, output_selected=false;
    const char *output = "photo.ppm", *meta = "capture.jsonl", *raw_path = NULL, *stats_path = NULL,
               *diagnostic = NULL, *camera_id=NULL;
    int opt, ret = 0;
    while ((opt = getopt_long(argc, argv, "c:I:o:h", opts, NULL)) != -1) {
        switch (opt) {
        case 'c':
            index_selected=true;
            ret = number(optarg, &index);
            break;
        case 'I': camera_id=optarg; break;
        case 'o':
            output = optarg;
            output_selected = true;
            break;
        case FLICKER:
            ret = !strcmp(optarg, "off") ? (tuning.mains_hz = 0, 0) : number(optarg, &tuning.mains_hz);
            break;
        case PROFILE:
            ret = cc_vc4_lookup_profile(optarg, &tuning.profile);
            break;
        case LIST_PROFILES:
            for (unsigned i = CC_VC4_PROFILE_OV5647; i <= CC_VC4_PROFILE_IMX708_WIDE_NOIR; ++i) {
                const cc_vc4_profile_info *p = cc_vc4_get_profile_info((enum cc_vc4_profile)i);
                printf("%s: sensor=%s calibration=%s%s\n", p->name, p->sensor_model, p->calibration_id,
                       p->flags & CC_VC4_PROFILE_GENERIC_LENS ? " (generic HQ lens calibration)" : "");
            }
            return 0;
        case TIMEOUT:
            ret = number(optarg, &timeout);
            break;
        case SHUTTER:
            ret = number(optarg, &shutter);
            break;
        case GAIN:
            gain_selected = true;
            ret = number(optarg, &gain);
            break;
        case WB: {
            char extra;
            ret = sscanf(optarg, "%lf,%lf%c", &tuning.red_gain, &tuning.blue_gain, &extra) == 2 ? 0 : -EINVAL;
            break;
        }
        case ED:
        case GD:
        case VD:
        case HD:
            delay_override[opt - ED] = true;
            ret = number(optarg, &delays[(unsigned)(opt - ED)]);
            break;
        case META:
            meta = optarg;
            break;
        case RAW:
            raw_path = optarg;
            break;
        case STATS:
            stats_path = optarg;
            break;
        case DIAG:
            diagnostic = optarg;
            break;
        case FRAMES:
            ret = number(optarg, &frames);
            break;
        case SIZE: {
            char extra;
            ret = sscanf(optarg, "%5ux%5u%c", &output_width, &output_height, &extra) == 2 &&
                output_width && output_height ? 0 : -EINVAL;
            break;
        }
        case REPEAT:
            ret = number(optarg, &repeat);
            if (!ret && !repeat)
                ret = -EINVAL;
            break;
        case HELP:
        case 'h':
            usage();
            return 0;
        default:
            usage();
            return 1;
        }
        if (ret < 0) {
            usage();
            return 1;
        }
    }
    if (optind != argc || (camera_id && (index_selected || !camera_id[0] ||
        strnlen(camera_id,CC_CAMERA_ID_MAX)==CC_CAMERA_ID_MAX)) || !timeout || timeout > 3600000 || frames < 32 || frames > 256 ||
        (repeat && (diagnostic || output_selected || raw_path || stats_path)) ||
        (diagnostic && strcmp(diagnostic, "exposure") && strcmp(diagnostic, "gain") &&
         strcmp(diagnostic, "vblank"))) {
        usage();
        return 1;
    }
    cc_ipa *validated = NULL;
    ret = cc_ipa_vc4_create(&tuning, &validated);
    cc_ipa_destroy(validated);
    if (ret < 0) {
        usage();
        return 1;
    }
    for (unsigned i = 0; i < 4; ++i)
        if (delays[i] > 64) {
            usage();
            return 1;
        }
    const char *paths[] = {diagnostic || repeat ? NULL : output, meta, raw_path, stats_path};
    for (unsigned i = 0; i < 4; ++i)
        for (unsigned j = 0; j < i; ++j)
            if (paths[i] && paths[j] && !strcmp(paths[i], paths[j])) {
                fputs("Output paths must differ.\n", stderr);
                return 1;
            }
    cc_camera_info *list = NULL;
    size_t count = 0;
    cc_camera *camera = NULL;
    cc_vc4_isp *isp = NULL;
    cc_capture *capture = NULL;
    cc_ipa *ipa = NULL;
    cc_request *request = NULL;
    FILE *raw_file = NULL;
    struct row rows[256];
    size_t row_count = 0;
    cc_sensor_trace traces[2048];
    size_t trace_count = 0;
    uint64_t trace_dropped = 0;
    cc_format raw = {0};
    enum cc_orientation orientation=CC_ORIENTATION_ROTATE_0;
    cc_vc4_configuration config = {0};
    cc_sensor_geometry geometry = {0};
    cc_sensor_timing timing = {0};
    const cc_vc4_profile_info *profile = NULL;
    FILE *repeat_log = NULL;
    int baseline_fds = 0;
    if (repeat) {
        repeat_log = fopen(meta, "w");
        if (!repeat_log) {
            ret = -errno;
            goto done;
        }
        baseline_fds = open_fds();
        if (baseline_fds < 0) {
            ret = baseline_fds;
            goto done;
        }
    }
    if (camera_id) ret=cc_camera_open_by_id(camera_id,&camera);
    else {
        ret=cc_enumerate(&list,&count);
        if (ret<0) goto done;
        if (index>=count) { ret=-ENODEV; goto done; }
        ret=cc_camera_open(&list[index],&camera);
    }
    if (ret<0) goto done;
    const cc_camera_info *source=cc_camera_get_info(camera);
    fprintf(stderr,"camera=%s driver=%s media=%s video=%s id=%s\n",source->name,source->driver,
            source->media_path,source->video_path,source->id[0] ? source->id:"(unavailable)");
    if (source->pipeline != CC_PIPELINE_RPI_UNICAM) {
        ret = -ENOTSUP;
        goto done;
    }
    const cc_sensor_model *model;
    ret = cc_camera_get_sensor(camera, &model);
    if (ret < 0)
        goto done;
    ret = cc_vc4_select_profile(model, tuning.profile, &tuning.profile);
    if (ret < 0)
        goto done;
    profile = cc_vc4_get_profile_info(tuning.profile);
    if (diagnostic && strcmp(profile->sensor_model, "ov5647")) {
        fputs("Timing diagnostics currently require OV5647.\n", stderr);
        ret = -ENOTSUP;
        goto done;
    }
    const cc_sensor_properties *sensor_properties = cc_sensor_get_properties(model);
    for (unsigned i = 0; i < sensor_properties->num_delays; ++i) {
        const uint32_t ids[] = {V4L2_CID_EXPOSURE, V4L2_CID_ANALOGUE_GAIN, V4L2_CID_VBLANK, V4L2_CID_HBLANK};
        for (unsigned j = 0; j < 4; ++j)
            if (!delay_override[j] && sensor_properties->delays[i].id == ids[j])
                delays[j] = sensor_properties->delays[i].frames;
    }
    cc_camera_properties properties;
    ret = cc_camera_get_properties(camera, &properties);
    if (ret < 0)
        goto done;
    if (!(properties.flags & CC_CAMERA_PROPERTY_ACTIVE_AREA)) {
        ret = -ENODATA;
        goto done;
    }
    uint32_t width = properties.active_area.width, height = properties.active_area.height;
    unsigned depth = !strcmp(profile->sensor_model, "imx477") ? 12U : 10U;
    const uint32_t codes10[] = {MEDIA_BUS_FMT_SBGGR10_1X10, MEDIA_BUS_FMT_SGBRG10_1X10,
                                MEDIA_BUS_FMT_SGRBG10_1X10, MEDIA_BUS_FMT_SRGGB10_1X10};
    const uint32_t codes12[] = {MEDIA_BUS_FMT_SBGGR12_1X12, MEDIA_BUS_FMT_SGBRG12_1X12,
                                MEDIA_BUS_FMT_SGRBG12_1X12, MEDIA_BUS_FMT_SRGGB12_1X12};
    cc_sensor_mode_request mode_request = {.mbus_codes = depth == 12 ? codes12 : codes10, .num_mbus_codes = 4,
        .width = width, .height = height, .max_width = width, .max_height = height};
    cc_sensor_format selected;
    ret = cc_camera_select_sensor_mode(camera, &mode_request, &selected);
    if (ret < 0)
        goto done;
    ret = cc_vc4_isp_open(NULL, &isp);
    if (ret < 0)
        goto done;
    cc_capture_devices planned_devices = {.camera = camera, .vc4 = isp};
    const enum cc_stream_role roles[] = {CC_ROLE_RAW, CC_ROLE_STILL_CAPTURE};
    cc_pipeline_configuration pipeline;
    ret = cc_pipeline_generate_configuration(&planned_devices, roles, 2, &pipeline);
    if (ret < 0)
        goto done;
    pipeline.sensor = selected; /* Require the calibrated full-resolution mode. */
    pipeline.streams[0].sensor_bit_depth = depth;
    pipeline.streams[1].format = (cc_format){.fourcc = V4L2_PIX_FMT_RGB24,
        .width = output_width ? output_width : width, .height = output_height ? output_height : height};
    pipeline.orientation = orientation;
    ret = cc_pipeline_configure(&planned_devices, &pipeline);
    if (ret < 0)
        goto done;
    raw = pipeline.source_format;
    orientation = pipeline.applied_orientation;
    ret = cc_vc4_isp_get_configuration(isp, &config);
    if (ret < 0)
        goto done;
    fprintf(stderr,"orientation=%u (EXIF), raw_fourcc=0x%08x\n",(unsigned)orientation,raw.fourcc);
    if (raw.width != width || raw.height != height) {
        ret = -EPROTO;
        goto done;
    }
    ret = disable_auto(camera);
    if (ret < 0)
        goto done;
    ret = cc_camera_get_sensor_geometry(camera, &geometry);
    if (ret < 0)
        goto done;
    ret = cc_camera_get_sensor_timing(camera, &timing);
    if (ret < 0)
        goto done;
    cc_sensor_timing_limits limits;
    ret = cc_camera_get_sensor_timing_limits(camera, &limits);
    if (ret < 0)
        goto done;
    uint32_t n = 4;
    ret = cc_vc4_isp_allocate_buffers(isp, &n);
    if (ret < 0)
        goto done;
    n = 4;
    ret = cc_camera_allocate_buffers(camera, &n);
    if (ret < 0)
        goto done;
    cc_capture_devices devices = {.camera = camera, .vc4 = isp};
    ret = cc_capture_create(&devices, &capture, &n);
    if (ret < 0)
        goto done;
    camera = NULL;
    isp = NULL;
    cc_sensor_control_delay d[4];
    size_t dn = 0;
    const uint32_t ids[] = {V4L2_CID_EXPOSURE, V4L2_CID_ANALOGUE_GAIN, V4L2_CID_VBLANK, V4L2_CID_HBLANK};
    for (unsigned i = 0; i < 4; ++i) {
        cc_control_info info;
        ret = cc_capture_query_control(capture, ids[i], &info);
        if (ret < 0)
            goto done;
        if (!(info.flags & V4L2_CTRL_FLAG_READ_ONLY))
            d[dn++] = (cc_sensor_control_delay){ids[i], delays[i], i == 2 ? CC_SENSOR_CONTROL_PRIORITY : 0};
    }
    ret = cc_capture_configure_sensor_delays(capture, d, dn);
    if (ret < 0)
        goto done;
    if (!repeat) {
        ret = cc_capture_enable_sensor_trace(capture, 1);
        if (ret < 0)
            goto done;
        ret = cc_request_create(1, &request);
        if (ret < 0)
            goto done;
    }
    if (!diagnostic) {
        cc_sensor_mode_policy p;
        uint64_t duration;
        ret = cc_sensor_frame_duration(&timing, &duration);
        if (ret < 0)
            goto done;
        cc_sensor_mode_description mode = {.width = raw.width, .height = raw.height, .binning_x = 1,
            .scale_x = 1, .min_frame_duration_ns = duration};
        ret = cc_sensor_get_mode_policy(model, &mode, &p);
        if (ret < 0)
            goto done;
        ret = cc_capture_configure_sensor_policy(capture, &p);
        if (ret < 0)
            goto done;
        ret = cc_ipa_vc4_create(&tuning, &ipa);
        if (!ret)
            ret = cc_capture_set_ipa(capture, &ipa);
        if (ret < 0)
            goto done;
        cc_control manual[2];
        size_t mn = 0;
        if (shutter) {
            uint32_t lines;
            ret = cc_sensor_exposure_lines(model, &timing, (uint64_t)shutter * 1000, &lines);
            if (ret < 0)
                goto done;
            manual[mn++] = (cc_control){.id = V4L2_CID_EXPOSURE, .value = lines};
        }
        if (gain_selected)
            manual[mn++] = (cc_control){.id = V4L2_CID_ANALOGUE_GAIN, .value = gain};
        if (repeat) {
            if (fprintf(repeat_log, "{\"type\":\"configuration\",\"raw_width\":%u,\"raw_height\":%u,"
                "\"rgb_width\":%u,\"rgb_height\":%u,\"profile\":\"%s\"}\n",
                raw.width, raw.height, config.output[0].width, config.output[0].height, profile->calibration_id) < 0)
                ret = -EIO;
            else
                ret = repeat_check(&capture, n, repeat, (int)timeout, manual, mn, repeat_log);
            goto done;
        }
        ret = cc_request_set_sensor_controls(request, manual, mn);
        if (ret < 0)
            goto done;
    } else {
        uint32_t exposure_lines;
        ret = cc_sensor_exposure_lines(model, &timing, 5000000, &exposure_lines);
        if (ret < 0)
            goto done;
        cc_control initial[] = {{.id = V4L2_CID_EXPOSURE, .value = exposure_lines},
                                {.id = V4L2_CID_ANALOGUE_GAIN, .value = 16}};
        /* Immediate writes must not bypass the active delay model. */
        ret = cc_capture_configure_sensor_delays(capture, NULL, 0);
        if (ret < 0)
            goto done;
        ret = cc_capture_set_controls(capture, initial, 2, NULL);
        if (ret < 0)
            goto done;
        ret = cc_capture_configure_sensor_delays(capture, d, dn);
        if (ret < 0)
            goto done;
        for (uint32_t target = 16; target < frames; target += 16) {
            cc_control change = {.id = !strcmp(diagnostic, "exposure") ? V4L2_CID_EXPOSURE
                                       : !strcmp(diagnostic, "gain")   ? V4L2_CID_ANALOGUE_GAIN
                                                                       : V4L2_CID_VBLANK};
            bool high = (target / 16) % 2 != 0;
            change.value = change.id == V4L2_CID_EXPOSURE ? (high ? exposure_lines * 3 : exposure_lines)
                           : change.id == V4L2_CID_ANALOGUE_GAIN
                               ? (high ? 64 : 16)
                               : timing.vblank + (high ? timing.height / 2 : 0);
            if (target > 120)
                break; /* Delay planner history is bounded to 128 frames. */
            ret = cc_capture_schedule_sensor_controls(capture, target, &change, 1, target);
            if (ret < 0)
                goto done;
        }
        cc_vc4_tuning baseline;
        ret = cc_vc4_build_tuning(tuning.profile, &geometry, 4500, 1, &baseline);
        if (!ret)
            ret = cc_request_set_vc4_tuning(request, &baseline);
        if (ret < 0)
            goto done;
        cc_control wb[] = {{.id = V4L2_CID_RED_BALANCE, .value = 1000},
                           {.id = V4L2_CID_BLUE_BALANCE, .value = 1000},
                           {.id = V4L2_CID_DIGITAL_GAIN, .value = 1000}};
        ret = cc_request_set_isp_controls(request, wb, 3);
        if (ret < 0)
            goto done;
    }
    if (raw_path) {
        raw_file = fopen(raw_path, "wb");
        if (!raw_file) {
            ret = -errno;
            goto done;
        }
    }
    uint64_t deadline = now_ns() + (uint64_t)timeout * 1000000;
    unsigned stable = 0;
    bool saved = false;
    while (row_count < 256) {
        if (now_ns() >= deadline) {
            ret = -ETIMEDOUT;
            break;
        }
        ret = cc_capture_queue(capture, request);
        if (ret < 0)
            break;
        cc_request *completed = NULL;
        uint64_t current = now_ns();
        uint64_t remaining = current < deadline ? deadline - current : 0;
        int wait = (int)(remaining / 1000000);
        if (wait < 1)
            wait = 1;
        ret = cc_capture_dequeue(capture, &completed, wait);
        if (ret < 0)
            break;
        const cc_capture_result *r = cc_request_get_result(completed);
        if (!r || r->error) {
            ret = r ? r->error : -EPROTO;
            break;
        }
        cc_ipa_statistics stats = {0};
        const cc_plane *sp = &r->statistics.planes[0];
        ret = cc_ipa_parse_statistics(config.statistics.fourcc,
                                      (const unsigned char *)sp->data + sp->data_offset,
                                      sp->bytes_used - sp->data_offset, &stats);
        if (ret < 0)
            break;
        rows[row_count++] = (struct row){.sequence = r->raw.sequence,
                                         .timestamp = r->raw.timestamp_ns,
                                         .flags = r->raw.flags,
                                         .ipa = r->ipa,
                                         .sensor = r->sensor,
                                         .luma = stats.luminance};
        uint64_t lost;
        size_t added;
        ret = cc_capture_read_sensor_trace(capture, traces + trace_count, 2048 - trace_count, &added, &lost);
        if (ret < 0)
            break;
        trace_count += added;
        trace_dropped += lost;
        bool ready = (r->ipa.flags & (CC_IPA_AE_CONVERGED | CC_IPA_AWB_CONVERGED)) ==
                         (CC_IPA_AE_CONVERGED | CC_IPA_AWB_CONVERGED) &&
                     !r->untrusted;
        stable = ready ? stable + 1 : 0;
        if (raw_file && (diagnostic || stable >= 3)) {
            ret = write_payload(raw_file, &r->raw);
            if (ret < 0)
                break;
        }
        if (!diagnostic && stable >= 3) {
            ret = ppm(output, &r->output[0], &config.output[0]);
            if (ret < 0)
                break;
            if (stats_path) {
                FILE *out = fopen(stats_path, "wb");
                if (!out) {
                    ret = -errno;
                    break;
                }
                ret = write_payload(out, &r->statistics);
                if (fclose(out) && !ret)
                    ret = -EIO;
                if (ret < 0)
                    break;
            }
            saved = true;
        }
        ret = cc_capture_release(capture, completed);
        if (ret < 0)
            break;
        if (saved || (diagnostic && row_count >= frames))
            break;
    }
    if (!ret && !diagnostic && !saved)
        ret = -ETIMEDOUT;
    if (capture) {
        int stopped = cc_capture_stop(capture);
        if (stopped < 0 && !ret)
            ret = stopped;
    }
    if (!ret)
        fprintf(stderr, "%s; %zu frames; profile=%s\n", diagnostic ? "Diagnostic complete" : output,
                row_count, profile->calibration_id);
done:
    if (capture) {
        uint64_t lost;
        size_t added;
        int drained =
            cc_capture_read_sensor_trace(capture, traces + trace_count, 2048 - trace_count, &added, &lost);
        if (!drained) {
            trace_count += added;
            trace_dropped += lost;
        }
    }
    cc_capture_close(capture);
    cc_request_destroy(request);
    cc_ipa_destroy(ipa);
    cc_vc4_isp_close(isp);
    cc_camera_close(camera);
    cc_camera_list_free(list);
    if (raw_file && fclose(raw_file) && !ret)
        ret = -EIO;
    if (repeat_log) {
        int final_fds = open_fds();
        if (!ret && final_fds != baseline_fds)
            ret = final_fds < 0 ? final_fds : -EIO;
        if (fprintf(repeat_log, "{\"type\":\"summary\",\"fds_before_open\":%d,\"fds_after_close\":%d,\"error\":%d}\n",
                    baseline_fds, final_fds, ret) < 0 && !ret)
            ret = -EIO;
        if (fclose(repeat_log) && !ret)
            ret = -EIO;
        if (!ret)
            fprintf(stderr, "Repeat check completed for %u captures after restart. See %s.\n", repeat, meta);
    } else if (!repeat) {
        int logged = metadata(meta, rows, row_count, traces, trace_count, trace_dropped, &raw,
                              &config.output[0], &geometry, delays, diagnostic, ret, &timing, orientation, profile);
        if (!ret)
            ret = logged;
    }
    if (ret < 0)
        fprintf(stderr, "Still capture failed: %s (%d). See metadata and Pi bring-up guide.\n",
                strerror(-ret), -ret);
    return ret < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}
