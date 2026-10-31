/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/capture.h>
#include <ccamera/pisp_be.h>

#include <errno.h>
#include <inttypes.h>
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
    unsigned long value = strtoul(text, &end, 10);
    if (errno || !text[0] || *end || text[0] == '-' || value > UINT32_MAX)
        return -EINVAL;
    *out = (uint32_t)value;
    return 0;
}

static int rectangle(const char *text, cc_rectangle *out)
{
    uint32_t values[4];
    const char *next = text;
    for (uint32_t i = 0; i < 4; ++i) {
        char *end;
        errno = 0;
        unsigned long value = strtoul(next, &end, 10);
        if (errno || end == next || next[0] == '-' || value > UINT32_MAX ||
            (i < 3 ? *end != ',' : *end != '\0')) return -EINVAL;
        values[i] = (uint32_t)value;
        next = i < 3 ? end + 1 : end;
    }
    if (values[0] > INT32_MAX || values[1] > INT32_MAX) return -EINVAL;
    *out = (cc_rectangle) { (int32_t)values[0], (int32_t)values[1], values[2], values[3] };
    return 0;
}

static int write_frame(FILE *file, const cc_frame *frame)
{
    if (!file)
        return 0;
    for (uint32_t i = 0; i < frame->num_planes; ++i) {
        const cc_plane *plane = &frame->planes[i];
        const unsigned char *bytes = plane->data;
        size_t count = plane->bytes_used - plane->data_offset;
        if (!bytes || fwrite(bytes + plane->data_offset, 1, count, file) != count)
            return -EIO;
    }
    return 0;
}

static void print_format(const char *name, const cc_format *format)
{
    fprintf(stderr, "%s: %ux%u fourcc=0x%08x (%u memory planes), color=%u encoding=%u range=%u transfer=%u\n",
            name, format->width, format->height, format->fourcc, format->num_planes,
            format->colorspace, format->ycbcr_enc, format->quantization, format->xfer_func);
    for (uint32_t p = 0; p < format->num_planes; ++p)
        fprintf(stderr, "  plane %u: stride=%u size=%u\n", p, format->planes[p].stride, format->planes[p].size);
}

static void usage(const char *program)
{
    fprintf(stderr, "Usage: %s [-c index] -w width -h height -f FOURCC\n"
            "          [-n frames] [-o bayer.raw] [-s statistics.bin] [-g] [-j]\n"
            "          [-b FE-FOURCC] [-k compression-black-level-offset]\n"
            "          [-p [-i /dev/mediaN] [-F FOURCC] [-A FOURCC] [-a auxiliary.raw]]\n"
            "          [-W width -H height] [-U aux-width -V aux-height]\n"
            "          [-r x,y,width,height] [-R aux-x,aux-y,aux-width,aux-height]\n"
            "          [-t transform] [-T aux-transform]\n"
            "Use ccamera-capture -l/-m to list cameras and sensor modes.\n"
            "Input FOURCC selects an 8/10/12-bit sensor mode. Output is full-size\n"
            "16-bit Bayer/mono plus native pisp_statistics records; no encoding.\n"
            "-b PC1R/PC1G/PC1g/PC1B selects mode 1 compressed Bayer (PC1M mono);\n"
            "PC2 selects mode 2 if supported by the kernel. Offset -k defaults to 0.\n"
            "-p adds PiSP BE demosaic (default full-size linear sensor RGB24).\n"
            "-F selects RGB/YUV output; -A/-a enables output1 (default YU12).\n"
            "-g adds reference automatic exposure, and white balance with -p.\n"
            "-j adds the standard V3/IMX708 continuous autofocus profile (opt-in).\n"
            "Transforms: 0 none, 1 horizontal flip, 2 vertical flip, 3 both.\n",
            program);
}

int main(int argc, char **argv)
{
    uint32_t index = 0, frames = 10;
    bool process = false, auxiliary = false, automatic = false, autofocus = false;
    cc_cfe_configuration configuration = {0};
    cc_format output_format = { .fourcc = CC_FOURCC('R', 'G', 'B', '3') };
    cc_format auxiliary_format = { .fourcc = CC_FOURCC('Y', 'U', '1', '2') };
    cc_rectangle crops[2] = {{0}};
    uint32_t transforms[2] = {0};
    const char *output_path = NULL, *statistics_path = NULL, *be_path = NULL, *auxiliary_path = NULL;
    int option;
    while ((option = getopt(argc, argv, "pgjc:w:h:f:b:k:n:o:s:i:F:A:a:W:H:U:V:r:R:t:T:")) != -1) {
        int ret = 0;
        switch (option) {
        case 'c': ret = number(optarg, &index); break;
        case 'w': ret = number(optarg, &configuration.sensor.width); break;
        case 'h': ret = number(optarg, &configuration.sensor.height); break;
        case 'n': ret = number(optarg, &frames); break;
        case 'k': ret = number(optarg, &configuration.compression_offset); break;
        case 'o': output_path = optarg; break;
        case 's': statistics_path = optarg; break;
        case 'p': process = true; break;
        case 'g': automatic = true; break;
        case 'j': autofocus = true; break;
        case 'i': process = true; be_path = optarg; break;
        case 'a': process = auxiliary = true; auxiliary_path = optarg; break;
        case 'W': process = true; ret = number(optarg, &output_format.width); break;
        case 'H': process = true; ret = number(optarg, &output_format.height); break;
        case 'U': process = auxiliary = true; ret = number(optarg, &auxiliary_format.width); break;
        case 'V': process = auxiliary = true; ret = number(optarg, &auxiliary_format.height); break;
        case 'r': process = true; ret = rectangle(optarg, &crops[0]); break;
        case 'R': process = auxiliary = true; ret = rectangle(optarg, &crops[1]); break;
        case 't': case 'T': {
            uint32_t branch = option == 'T' ? 1U : 0U;
            process = true;
            if (branch) auxiliary = true;
            ret = number(optarg, &transforms[branch]);
            if (transforms[branch] > 3) ret = -EINVAL;
            break;
        }
        case 'A':
        case 'F':
        case 'b':
        case 'f':
            if (strlen(optarg) != 4) { ret = -EINVAL; break; }
            uint32_t code = CC_FOURCC((unsigned char)optarg[0], (unsigned char)optarg[1],
                                      (unsigned char)optarg[2], (unsigned char)optarg[3]);
            if (option == 'A') { process = auxiliary = true; auxiliary_format.fourcc = code; }
            else if (option == 'F') { process = true; output_format.fourcc = code; }
            else if (option == 'b') configuration.image.fourcc = code;
            else configuration.sensor.fourcc = code;
            break;
        default: usage(argv[0]); return EXIT_FAILURE;
        }
        if (ret < 0) { usage(argv[0]); return EXIT_FAILURE; }
    }
    if (optind != argc || !configuration.sensor.width || !configuration.sensor.height ||
        !configuration.sensor.fourcc || !frames || configuration.compression_offset>UINT16_MAX ||
        (output_path && statistics_path && !strcmp(output_path, statistics_path)) ||
        (auxiliary_path && output_path && !strcmp(auxiliary_path, output_path)) ||
        (auxiliary_path && statistics_path && !strcmp(auxiliary_path, statistics_path))) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    cc_camera_info *cameras = NULL;
    size_t count = 0;
    cc_cfe *cfe = NULL;
    cc_pisp_be *be = NULL;
    cc_capture *capture = NULL;
    cc_ipa *ipa = NULL;
    cc_request **requests = NULL;
    uint32_t request_count = 0;
    uint32_t be_buffers = 4;
    FILE *output = NULL, *statistics = NULL, *auxiliary_file = NULL;
    int ret = cc_enumerate(&cameras, &count);
    if (ret < 0)
        goto done;
    if (index >= count) { ret = -ENODEV; goto done; }
    ret = cc_cfe_open(&cameras[index], &cfe);
    if (ret < 0)
        goto done;
    ret = cc_cfe_configure(cfe, &configuration);
    if (ret < 0)
        goto done;
    const cc_format *image = &configuration.image;
    fprintf(stderr, "%s FE: %ux%u fourcc=0x%08x stride=%u size=%u; statistics=%u bytes\n",
            cameras[index].name, image->width, image->height, image->fourcc,
            image->planes[0].stride, image->planes[0].size, configuration.statistics.planes[0].size);
    uint32_t buffers = 4;
    ret = cc_cfe_allocate_buffers(cfe, &buffers);
    if (ret < 0)
        goto done;
    if (process) {
        ret = cc_pisp_be_open(be_path, &be);
        if (ret < 0) goto done;
        cc_pisp_be_configuration c = { .input = configuration.image, .output = output_format,
            .compression_offset = configuration.compression_offset,
            .output1 = auxiliary ? auxiliary_format : (cc_format) {0},
            .crop = {crops[0], crops[1]}, .transform = {transforms[0], transforms[1]} };
        ret = cc_pisp_be_configure(be, &c);
        if (ret < 0) goto done;
        print_format("BE output0", &c.output);
        fprintf(stderr, "  crop=%d,%d/%ux%u transform=%u\n", c.crop[0].x, c.crop[0].y,
                c.crop[0].width, c.crop[0].height, c.transform[0]);
        if (auxiliary) print_format("BE output1", &c.output1);
        if (auxiliary) fprintf(stderr, "  crop=%d,%d/%ux%u transform=%u\n", c.crop[1].x, c.crop[1].y,
                               c.crop[1].width, c.crop[1].height, c.transform[1]);
        ret = cc_pisp_be_allocate_buffers(be, &be_buffers);
        if (ret < 0) goto done;
    }
    cc_capture_devices devices = { .frontend = cfe, .backend = be };
    ret = cc_capture_create(&devices, &capture, &request_count);
    if (ret < 0) goto done;
    cfe = NULL; be = NULL; /* The request graph now owns both devices. */
    if (automatic || autofocus) {
        ret = automatic ? cc_capture_configure_sensor_defaults(capture):0;
        cc_af_tuning af; cc_af_reference_tuning(&af);
        if (!ret && autofocus) {
            const cc_sensor_model *sensor;
            ret=cc_capture_get_sensor(capture,&sensor);
            if (!ret && strcmp(cc_sensor_get_properties(sensor)->model,"imx708")) ret=-ENOTSUP;
        }
        cc_ipa_reference_tuning tuning = {.flags=(automatic ? CC_IPA_REFERENCE_AE | (process ? CC_IPA_REFERENCE_AWB:0U):0U) |
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
        if (!output) { ret = -errno; goto done; }
    }
    if (statistics_path) {
        statistics = !strcmp(statistics_path, "-") ? stdout : fopen(statistics_path, "wb");
        if (!statistics) { ret = -errno; goto done; }
    }
    if (auxiliary_path) {
        auxiliary_file = !strcmp(auxiliary_path, "-") ? stdout : fopen(auxiliary_path, "wb");
        if (!auxiliary_file) { ret = -errno; goto done; }
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
        if (ret >= 0 && auxiliary) ret = write_frame(auxiliary_file, &result->output[1]);
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
    /* The graph releases downstream imports before source allocations. */
    cc_capture_close(capture);
    cc_ipa_destroy(ipa);
    if (requests) for (uint32_t i = 0; i < request_count; ++i) cc_request_destroy(requests[i]);
    free(requests);
    cc_pisp_be_close(be);
    cc_cfe_close(cfe);
    cc_camera_list_free(cameras);
    if (output && (output == stdout ? fflush(output) : fclose(output)) && ret >= 0)
        ret = -EIO;
    if (statistics && (statistics == stdout ? fflush(statistics) : fclose(statistics)) && ret >= 0)
        ret = -EIO;
    if (auxiliary_file && (auxiliary_file == stdout ? fflush(auxiliary_file) : fclose(auxiliary_file)) && ret >= 0)
        ret = -EIO;
    if (ret < 0)
        fprintf(stderr, "CFE capture failed: %s (%d)\n", strerror(-ret), -ret);
    return ret < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}
