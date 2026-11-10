/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/capture.h>
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <linux/media-bus-format.h>
#include <linux/videodev2.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/utsname.h>
#include <time.h>

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
static uint32_t bayer(uint32_t code)
{
    switch (code) {
    case MEDIA_BUS_FMT_SBGGR10_1X10:
        return V4L2_PIX_FMT_SBGGR10P;
    case MEDIA_BUS_FMT_SGBRG10_1X10:
        return V4L2_PIX_FMT_SGBRG10P;
    case MEDIA_BUS_FMT_SGRBG10_1X10:
        return V4L2_PIX_FMT_SGRBG10P;
    case MEDIA_BUS_FMT_SRGGB10_1X10:
        return V4L2_PIX_FMT_SRGGB10P;
    default:
        return 0;
    }
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
                    int status, const cc_sensor_timing *timing)
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
            "u],\"delays\":[%u,%u,%u,%u],\"trace_dropped\":%" PRIu64 "}\n",
            cc_vc4_ov5647_profile_id(), diagnostic ? diagnostic : "", status, raw->fourcc, raw->width,
            raw->height, raw->planes[0].stride, rgb->planes[0].stride, geometry->crop.x, geometry->crop.y,
            geometry->crop.width, geometry->crop.height, delays[0], delays[1], delays[2], delays[3], dropped);
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
static void usage(void)
{
    puts("ccamera-still [-c INDEX] [-o photo.ppm] [--metadata run.jsonl] [--raw frame.raw]\n"
         "  [--statistics frame.bsta] [--flicker off|50|60] [--timeout-ms 10000]\n"
         "  [--shutter-us N] [--gain-code N] [--colour-gains R,B]\n"
         "  [--exposure-delay N] [--gain-delay N] [--vblank-delay N] [--hblank-delay N]\n"
         "  [--diagnostic exposure|gain|vblank --frames 96]\n"
         "Standard colour OV5647 on Pi 4/earlier only. Uses full-resolution mode.\n"
         "Diagnostics use fixed illumination and settings; logs are written after capture.\n"
         "Run ccamera-capture -l to choose the discovery index.");
}
int main(int argc, char **argv)
{
    enum { FLICKER = 256, TIMEOUT, SHUTTER, GAIN, WB, ED, GD, VD, HD, META, RAW, STATS, DIAG, FRAMES, HELP };
    const struct option opts[] = {{"flicker", 1, 0, FLICKER},
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
                                  {"help", 0, 0, HELP},
                                  {0, 0, 0, 0}};
    uint32_t index = 0, timeout = 10000, shutter = 0, gain = 0, delays[4] = {2, 2, 2, 2}, frames = 96;
    cc_ipa_vc4_options tuning = {.mains_hz = 50};
    const char *output = "photo.ppm", *meta = "capture.jsonl", *raw_path = NULL, *stats_path = NULL,
               *diagnostic = NULL;
    int opt, ret = 0;
    while ((opt = getopt_long(argc, argv, "c:o:h", opts, NULL)) != -1) {
        switch (opt) {
        case 'c':
            ret = number(optarg, &index);
            break;
        case 'o':
            output = optarg;
            break;
        case FLICKER:
            ret = !strcmp(optarg, "off") ? (tuning.mains_hz = 0, 0) : number(optarg, &tuning.mains_hz);
            break;
        case TIMEOUT:
            ret = number(optarg, &timeout);
            break;
        case SHUTTER:
            ret = number(optarg, &shutter);
            break;
        case GAIN:
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
    if (optind != argc || !timeout || timeout > 3600000 || frames < 32 || frames > 256 ||
        (diagnostic && strcmp(diagnostic, "exposure") && strcmp(diagnostic, "gain") &&
         strcmp(diagnostic, "vblank"))) {
        usage();
        return 1;
    }
    cc_ipa *validated = NULL;
    ret = cc_ipa_vc4_create(&tuning, &validated);
    cc_ipa_destroy(validated);
    if (ret < 0 || (gain && (gain < 16 || gain > 1023))) {
        usage();
        return 1;
    }
    for (unsigned i = 0; i < 4; ++i)
        if (delays[i] > 64) {
            usage();
            return 1;
        }
    const char *paths[] = {diagnostic ? NULL : output, meta, raw_path, stats_path};
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
    cc_vc4_configuration config = {0};
    cc_sensor_geometry geometry = {0};
    cc_sensor_timing timing = {0};
    ret = cc_enumerate(&list, &count);
    if (ret < 0)
        goto done;
    if (index >= count) {
        ret = -ENODEV;
        goto done;
    }
    fprintf(stderr, "camera=%s driver=%s media=%s video=%s\n", list[index].name, list[index].driver,
            list[index].media_path, list[index].video_path);
    if (list[index].pipeline != CC_PIPELINE_RPI_UNICAM) {
        ret = -ENOTSUP;
        goto done;
    }
    ret = cc_camera_open(&list[index], &camera);
    if (ret < 0)
        goto done;
    const cc_sensor_model *model;
    ret = cc_camera_get_sensor(camera, &model);
    if (ret < 0)
        goto done;
    if (strcmp(cc_sensor_get_properties(model)->model, "ov5647")) {
        ret = -ENOTSUP;
        goto done;
    }
    const uint32_t codes[] = {MEDIA_BUS_FMT_SBGGR10_1X10, MEDIA_BUS_FMT_SGBRG10_1X10,
                              MEDIA_BUS_FMT_SGRBG10_1X10, MEDIA_BUS_FMT_SRGGB10_1X10};
    cc_sensor_mode_request mode_request = {.mbus_codes = codes, .num_mbus_codes = 4,
        .width = 2592, .height = 1944, .max_width = 2592, .max_height = 1944};
    cc_sensor_format selected;
    ret = cc_camera_select_sensor_mode(camera, &mode_request, &selected);
    if (ret < 0)
        goto done;
    raw = (cc_format){.fourcc = bayer(selected.mbus_code), .width = selected.width, .height = selected.height};
    ret = cc_camera_configure(camera, &raw);
    if (ret < 0)
        goto done;
    if (raw.width != 2592 || raw.height != 1944) {
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
    ret = cc_vc4_isp_open(NULL, &isp);
    if (ret < 0)
        goto done;
    config = (cc_vc4_configuration){
        .input = raw, .output = {{.fourcc = V4L2_PIX_FMT_RGB24, .width = raw.width, .height = raw.height}}};
    ret = cc_vc4_isp_configure(isp, &config);
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
    ret = cc_capture_enable_sensor_trace(capture, 1);
    if (ret < 0)
        goto done;
    ret = cc_request_create(1, &request);
    if (ret < 0)
        goto done;
    if (!diagnostic) {
        cc_sensor_mode_policy p = {
            .hide_startup = 2, .hide_mode_switch = 2, .mistrust_startup = 2, .mistrust_mode_switch = 2};
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
        if (gain)
            manual[mn++] = (cc_control){.id = V4L2_CID_ANALOGUE_GAIN, .value = gain};
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
        ret = cc_vc4_ov5647_build_tuning(&geometry, 4500, 1, &baseline);
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
                row_count, cc_vc4_ov5647_profile_id());
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
    {
        int logged = metadata(meta, rows, row_count, traces, trace_count, trace_dropped, &raw,
                              &config.output[0], &geometry, delays, diagnostic, ret, &timing);
        if (!ret)
            ret = logged;
    }
    if (ret < 0)
        fprintf(stderr, "Still capture failed: %s (%d). See metadata and Pi bring-up guide.\n",
                strerror(-ret), -ret);
    return ret < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}
