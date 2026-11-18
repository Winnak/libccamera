/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "ppm.h"
#include <ccamera/pisp_tuning.h>
#include <ccamera/still.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void usage(FILE *out, const char *program)
{
    fprintf(out, "Usage: %s [-c INDEX | -I ID] [-W WIDTH -H HEIGHT] [-p PROFILE] output.ppm\n", program);
}

static int number(const char *text, uint32_t *value)
{
    if (text[0] < '0' || text[0] > '9')
        return -EINVAL;
    char *end;
    errno = 0;
    unsigned long parsed = strtoul(text, &end, 10);
    if (errno || *end || parsed > UINT32_MAX)
        return -EINVAL;
    *value = (uint32_t)parsed;
    return 0;
}

static int open_camera(uint32_t index, const char *id, const cc_still_options *options,
                       cc_capture **capture, cc_still_configuration *configuration)
{
    if (id)
        return cc_still_open_by_id(id, options, capture, configuration);
    cc_camera_info *list = NULL;
    size_t count = 0;
    int ret = cc_enumerate(&list, &count);
    if (!ret)
        ret = index < count ? cc_still_open(&list[index], options, capture, configuration) : -ENODEV;
    cc_camera_list_free(list);
    return ret;
}

static int photograph(cc_capture *capture, cc_request *request,
                      const cc_still_configuration *configuration, const char *path)
{
    const uint32_t converged = CC_IPA_AE_CONVERGED | CC_IPA_AWB_CONVERGED;
    /* Give the calibrated controllers time to settle, with a finite frame budget. */
    for (unsigned i = 0; ; ++i) {
        cc_request *completed = NULL;
        int ret = cc_capture_queue(capture, request);
        if (!ret)
            ret = cc_capture_dequeue(capture, &completed, 5000);
        if (ret < 0)
            return ret;
        const cc_capture_result *result = cc_request_get_result(completed);
        if (result->status != CC_REQUEST_COMPLETE)
            return result->error;
        if (!(result->ipa.flags & CC_IPA_METADATA_PHOTO) ||
            (result->ipa.flags & converged) == converged || i == 15) {
            const cc_frame *image = &result->raw;
            if (configuration->stream != CC_CAPTURE_RAW)
                image = &result->output[configuration->stream == CC_CAPTURE_OUTPUT1 ? 1 : 0];
            ret = ppm(path, image, &configuration->format);
            return ret < 0 ? ret : cc_capture_release(capture, completed);
        }
        ret = cc_capture_release(capture, completed);
        if (ret < 0)
            return ret;
    }
}

int main(int argc, char **argv)
{
    cc_still_options options = {.fourcc = CC_FOURCC('R', 'G', 'B', '3')};
    uint32_t index = 0;
    const char *id = NULL;
    int indexed = 0, opt, ret = 0;
    while (!ret && (opt = getopt(argc, argv, "c:I:W:H:p:h")) != -1) {
        switch (opt) {
        case 'c': indexed = 1; ret = number(optarg, &index); break;
        case 'I': id = optarg; break;
        case 'W': ret = number(optarg, &options.width); if (!options.width) ret = -EINVAL; break;
        case 'H': ret = number(optarg, &options.height); if (!options.height) ret = -EINVAL; break;
        case 'p': ret = cc_pisp_lookup_profile(optarg, &options.profile); break;
        case 'h': usage(stdout, argv[0]); return 0;
        default: ret = -EINVAL; break;
        }
    }
    if (ret < 0 || argc - optind != 1 || (indexed && id) ||
        ((options.width == 0) != (options.height == 0))) {
        usage(stderr, argv[0]);
        return 2;
    }

    cc_capture *capture = NULL;
    cc_request *request = NULL;
    cc_still_configuration configuration;
    ret = open_camera(index, id, &options, &capture, &configuration);
    if (!ret && configuration.format.fourcc != options.fourcc) {
        fputs("The camera did not negotiate RGB24 for PPM output.\n", stderr);
        ret = -ENOTSUP;
    }
    if (!ret)
        ret = cc_request_create(0, &request);
    if (!ret)
        ret = photograph(capture, request, &configuration, argv[optind]);
    cc_capture_close(capture);
    cc_request_destroy(request);
    if (ret < 0) {
        fprintf(stderr, "Photo capture failed (%s).\n", strerror(-ret));
        return 1;
    }
    return 0;
}
