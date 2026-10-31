/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/capture.h>
#include <ccamera/vc4_isp.h>

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static volatile sig_atomic_t interrupted;
static void on_signal(int signo) { (void)signo; interrupted = 1; }

static int number(const char *text, uint32_t *out)
{
    char *end;
    errno = 0;
    unsigned long n = strtoul(text, &end, 10);
    if (errno || !text[0] || *end || text[0] == '-' || n > UINT32_MAX)
        return -EINVAL;
    *out = (uint32_t)n;
    return 0;
}

static int pixel_format(const char *text, uint32_t *out)
{
    if (strlen(text) != 4)
        return -EINVAL;
    *out = CC_FOURCC((unsigned char)text[0], (unsigned char)text[1], (unsigned char)text[2], (unsigned char)text[3]);
    return 0;
}

static int write_frame(FILE *file, const cc_frame *frame)
{
    if (!file)
        return 0;
    for (uint32_t i = 0; i < frame->num_planes; ++i) {
        const cc_plane *p = &frame->planes[i];
        const unsigned char *bytes = p->data;
        size_t count = p->bytes_used - p->data_offset;
        if (!bytes || fwrite(bytes + p->data_offset, 1, count, file) != count)
            return -EIO;
    }
    return 0;
}

static void usage(const char *program)
{
    fprintf(stderr, "Usage: %s -l\n"
            "       %s [-c index | -d /dev/videoN] -m\n"
            "       %s [-c index | -d /dev/videoN] -w width -h height -f FOURCC\n"
            "          [-n frames] [-o output.raw] [-g] [-j]\n"
            "          [-p [-i /dev/mediaN] [-W output-width -H output-height -F FOURCC]\n"
            "              [-s statistics.bin]]\n"
            "The output concatenates each frame's plane payloads without encoding.\n"
            "-p enables the Pi 4/earlier ISP (default output: 640x480 RGB3).\n"
            "-g enables that ISP with reference automatic exposure/white balance.\n"
            "-j adds the standard V3/IMX708 continuous autofocus profile (opt-in).\n"
            "Use -c for CSI/VIMC cameras so the sensor and media route are configured.\n",
            program, program, program);
}

static const char *pipeline_name(enum cc_pipeline pipeline)
{
    switch (pipeline) {
    case CC_PIPELINE_RPI_UNICAM: return "rpi-unicam";
    case CC_PIPELINE_RPI_CFE: return "rpi-cfe";
    case CC_PIPELINE_VIMC: return "vimc";
    case CC_PIPELINE_UVC: return "uvc";
    default: return "v4l2";
    }
}

static int print_modes(cc_camera *camera)
{
    for (uint32_t i = 0;; ++i) {
        cc_format_info format;
        int ret = cc_camera_enum_format(camera, i, &format);
        if (ret == -ENOENT)
            break;
        if (ret < 0)
            return ret;
        printf("capture: %c%c%c%c %s\n", (int)(format.fourcc & 0xff),
                (int)((format.fourcc >> 8) & 0xff), (int)((format.fourcc >> 16) & 0xff),
                (int)((format.fourcc >> 24) & 0xff), format.description);
    }
    if (cc_camera_get_info(camera)->pipeline == CC_PIPELINE_V4L2 ||
        cc_camera_get_info(camera)->pipeline == CC_PIPELINE_UVC)
        return 0;
    for (uint32_t code = 0;; ++code) {
        cc_sensor_mode mode;
        int ret = cc_camera_enum_sensor_mode(camera, code, 0, &mode);
        if (ret == -ENOENT)
            return 0;
        if (ret < 0)
            return ret;
        for (uint32_t size = 0;; ++size) {
            ret = cc_camera_enum_sensor_mode(camera, code, size, &mode);
            if (ret == -ENOENT)
                break;
            if (ret < 0)
                return ret;
            printf("sensor: mbus=0x%04x width=%u..%u height=%u..%u\n", mode.mbus_code,
                    mode.sizes.min_width, mode.sizes.max_width, mode.sizes.min_height, mode.sizes.max_height);
        }
    }
}

int main(int argc, char **argv)
{
    bool list_only = false, modes_only = false, process = false, automatic = false, autofocus = false;
    uint32_t index = 0, frames = 10;
    cc_format format = {0};
    cc_format output_format = { .fourcc = CC_FOURCC('R', 'G', 'B', '3'), .width = 640, .height = 480 };
    const char *video_path = NULL, *output_path = NULL, *isp_path = NULL, *statistics_path = NULL;
    int option;
    while ((option = getopt(argc, argv, "lmpgjc:d:w:h:f:n:o:i:W:H:F:s:")) != -1) {
        int ret = 0;
        switch (option) {
        case 'l': list_only = true; break;
        case 'm': modes_only = true; break;
        case 'p': process = true; break;
        case 'g': automatic = process = true; break;
        case 'j': autofocus = process = true; break;
        case 'i': process = true; isp_path = optarg; break;
        case 'W': process = true; ret = number(optarg, &output_format.width); break;
        case 'H': process = true; ret = number(optarg, &output_format.height); break;
        case 'F': process = true; ret = pixel_format(optarg, &output_format.fourcc); break;
        case 's': process = true; statistics_path = optarg; break;
        case 'c': ret = number(optarg, &index); break;
        case 'd': video_path = optarg; break;
        case 'w': ret = number(optarg, &format.width); break;
        case 'h': ret = number(optarg, &format.height); break;
        case 'n': ret = number(optarg, &frames); break;
        case 'o': output_path = optarg; break;
        case 'f': ret = pixel_format(optarg, &format.fourcc); break;
        default: usage(argv[0]); return EXIT_FAILURE;
        }
        if (ret < 0) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }
    if (optind != argc || (!list_only && !modes_only &&
        (!format.width || !format.height || !format.fourcc || !frames ||
         (process && (!output_format.width || !output_format.height)) ||
         (output_path && statistics_path && !strcmp(output_path, statistics_path))))) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }
    cc_camera_info *list = NULL;
    size_t count = 0;
    cc_camera *camera = NULL;
    cc_vc4_isp *isp = NULL;
    cc_capture *capture = NULL;
    cc_ipa *ipa = NULL;
    cc_request **requests = NULL;
    uint32_t request_count = 0;
    uint32_t isp_buffers = 4;
    FILE *output = NULL, *statistics = NULL;
    int ret = 0;
    if (list_only || !video_path) {
        ret = cc_enumerate(&list, &count);
        if (ret < 0)
            goto done;
    }
    if (list_only) {
        for (size_t i = 0; i < count; ++i)
            printf("%zu: %s [%s] %s\n", i, list[i].name,
                    pipeline_name(list[i].pipeline),
                    list[i].video_path);
        if (!count)
            puts("No accessible supported cameras found.");
        goto done;
    }
    if (video_path) {
        ret = cc_camera_open_video(video_path, &camera);
    } else if (index < count) {
        ret = cc_camera_open(&list[index], &camera);
    } else {
        ret = -ENODEV;
    }
    if (ret < 0)
        goto done;
    if (modes_only) {
        ret = print_modes(camera);
        goto done;
    }
    ret = cc_camera_configure(camera, &format);
    if (ret < 0)
        goto done;
    char fourcc[5] = {0};
    for (unsigned int i = 0; i < 4; ++i)
        fourcc[i] = (char)((format.fourcc >> (i * 8)) & 0xff);
    fprintf(stderr, "%s: %ux%u %s (%u memory planes)\n",
            cc_camera_get_info(camera)->name, format.width, format.height, fourcc, format.num_planes);
    for (uint32_t i = 0; i < format.num_planes; ++i)
        fprintf(stderr, "  plane %u: stride=%u size=%u\n", i, format.planes[i].stride, format.planes[i].size);
    if (process) {
        ret = cc_vc4_isp_open(isp_path, &isp);
        if (ret < 0)
            goto done;
        cc_vc4_configuration c = { .input = format, .output = {output_format} };
        ret = cc_vc4_isp_configure(isp, &c);
        if (ret < 0)
            goto done;
        fprintf(stderr, "ISP main: %ux%u fourcc=0x%08x stride=%u size=%u, crop=%d,%d/%ux%u\n",
                c.output[0].width, c.output[0].height, c.output[0].fourcc, c.output[0].planes[0].stride,
                c.output[0].planes[0].size, c.crop.x, c.crop.y, c.crop.width, c.crop.height);
        ret = cc_vc4_isp_allocate_buffers(isp, &isp_buffers);
        if (ret < 0)
            goto done;
    }
    uint32_t buffers = 4;
    ret = cc_camera_allocate_buffers(camera, &buffers);
    if (ret < 0)
        goto done;
    cc_capture_devices devices = { .camera = camera, .vc4 = isp };
    ret = cc_capture_create(&devices, &capture, &request_count);
    if (ret < 0) goto done;
    camera = NULL; isp = NULL;
    if (automatic || autofocus) {
        ret = automatic ? cc_capture_configure_sensor_defaults(capture):0;
        cc_af_tuning af; cc_af_reference_tuning(&af);
        if (!ret && autofocus) {
            const cc_sensor_model *sensor;
            ret=cc_capture_get_sensor(capture,&sensor);
            if (!ret && strcmp(cc_sensor_get_properties(sensor)->model,"imx708")) ret=-ENOTSUP;
        }
        cc_ipa_reference_tuning tuning = {.flags=(automatic ? CC_IPA_REFERENCE_AE | CC_IPA_REFERENCE_AWB:0U) |
                (autofocus ? CC_IPA_REFERENCE_AF:0U),.af=autofocus ? &af:NULL,
            .target_luminance=0.18,.exposure_speed=0.25,.white_balance_speed=0.25,
            .min_colour_gain=0.5,.max_colour_gain=4};
        if (!ret) ret = cc_ipa_reference_create(&tuning, &ipa);
        if (!ret) ret = cc_capture_set_ipa(capture, &ipa);
        if (ret < 0) goto done;
    }
    if (request_count > frames) request_count = frames;
    requests = calloc(request_count, sizeof(*requests));
    if (!requests) { ret = -ENOMEM; goto done; }
    for (uint32_t i = 0; i < request_count; ++i) {
        ret = cc_request_create(i, &requests[i]);
        if (ret>=0 && autofocus) {
            cc_af_controls af={.flags=CC_AF_SET_MODE,.mode=CC_AF_CONTINUOUS};
            ret=cc_request_set_af_controls(requests[i],&af);
        }
        if (ret >= 0) ret = cc_capture_queue(capture, requests[i]);
        if (ret < 0) goto done;
    }
    if (output_path) {
        output = !strcmp(output_path, "-") ? stdout : fopen(output_path, "wb");
        if (!output) {
            ret = -errno;
            goto done;
        }
    }
    if (statistics_path) {
        statistics = !strcmp(statistics_path, "-") ? stdout : fopen(statistics_path, "wb");
        if (!statistics) { ret = -errno; goto done; }
    }
    struct sigaction action = { .sa_handler = on_signal };
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    uint64_t next_request = request_count;
    for (uint32_t i = 0; i < frames && !interrupted; ++i) {
        cc_request *request;
        ret = cc_capture_dequeue(capture, &request, 3000);
        if (ret < 0) goto done;
        const cc_capture_result *result = cc_request_get_result(request);
        if (result->error) { ret = result->error; goto done; }
        fprintf(stderr, "cookie=%" PRIu64 " sequence=%u timestamp=%" PRIu64 " flags=0x%x\n",
                result->cookie, result->raw.sequence, result->raw.timestamp_ns, result->raw.flags);
        if (result->ipa.flags & CC_IPA_METADATA_AE)
            fprintf(stderr,"  exposure=%" PRIu64 "ns gain=%.3f luma=%.4f\n",result->ipa.exposure_ns,
                    result->ipa.analogue_gain,result->ipa.luminance);
        if (result->ipa.flags & CC_IPA_METADATA_AWB)
            fprintf(stderr,"  white-balance red=%.3f blue=%.3f\n",result->ipa.red_gain,result->ipa.blue_gain);
        if (result->ipa.flags & CC_IPA_METADATA_AF)
            fprintf(stderr,"  autofocus state=%u lens=%d position=%.3f dioptres\n",(unsigned int)result->ipa.af.state,
                result->ipa.af.lens_setting,result->ipa.af.lens_position);
        ret = write_frame(output, process ? &result->output[0] : &result->raw);
        if (ret >= 0) ret = write_frame(statistics, &result->statistics);
        if (ret >= 0) ret = cc_capture_release(capture, request);
        if (ret < 0) goto done;
        if (next_request < frames && !interrupted) {
            ret = cc_request_set_cookie(request, next_request++);
            if (ret >= 0) ret = cc_capture_queue(capture, request);
            if (ret < 0) goto done;
        }
    }
    ret = cc_capture_stop(capture);
done:
    cc_capture_close(capture);
    cc_ipa_destroy(ipa);
    if (requests) for (uint32_t i = 0; i < request_count; ++i) cc_request_destroy(requests[i]);
    free(requests);
    cc_vc4_isp_close(isp);
    cc_camera_close(camera);
    cc_camera_list_free(list);
    if (output && (output == stdout ? fflush(output) : fclose(output)) && ret >= 0)
        ret = -EIO;
    if (statistics && (statistics == stdout ? fflush(statistics) : fclose(statistics)) && ret >= 0)
        ret = -EIO;
    if (ret < 0)
        fprintf(stderr, "Capture failed: %s (%d)\n", strerror(-ret), -ret);
    return ret < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}
