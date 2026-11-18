/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include <ccamera/configuration.h>
#include <ccamera/still.h>
#include <errno.h>

static int configure_pi(const cc_capture_devices *devices, const cc_still_options *options,
                        cc_still_configuration *still)
{
    cc_format format = {
        .fourcc = options->fourcc ? options->fourcc : V4L2_PIX_FMT_YUV420,
        .width = options->width,
        .height = options->height,
    };
    if (!format.width) {
        cc_camera *source = devices->camera ? devices->camera : cc_cfe_source(devices->frontend);
        cc_camera_properties properties = {0};
        int ret = cc_sensor_mode_properties(source, 0, &properties);
        if (ret < 0)
            return ret;
        format.width = properties.pixel_array_width;
        format.height = properties.pixel_array_height;
        if (!format.width || !format.height)
            return -ENODATA;
    }
    cc_pipeline_configuration configuration = {
        .num_streams = 1,
        .streams = {{.role = CC_ROLE_STILL_CAPTURE, .format = format, .buffer_count = still->buffer_count}},
        .orientation = CC_ORIENTATION_ROTATE_0,
    };
    int ret = cc_pipeline_configure(devices, &configuration);
    if (ret < 0)
        return ret;
    const cc_stream_configuration *image = &configuration.streams[0];
    if (image->stream == CC_CAPTURE_RAW)
        return -ENOTSUP;
    still->format = image->format;
    still->stream = image->stream;
    still->orientation = configuration.applied_orientation;
    return 0;
}

static int configure_native(cc_camera *camera, const cc_still_options *options,
                            cc_still_configuration *still)
{
    cc_format current = {0};
    if (!options->fourcc || !options->width) {
        int ret = cc_camera_get_format(camera, &current);
        if (ret < 0)
            return ret;
    }
    still->format = (cc_format) {
        .fourcc = options->fourcc ? options->fourcc : current.fourcc,
        .width = options->width ? options->width : current.width,
        .height = options->height ? options->height : current.height,
    };
    still->stream = CC_CAPTURE_RAW;
    return cc_camera_configure(camera, &still->format);
}

int cc_still_open_io(const cc_camera_info *info, const cc_still_options *options, const struct cc_io *io,
                     cc_capture **out, cc_still_configuration *configuration)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    if (!info || !configuration)
        return -EINVAL;
    const cc_still_options defaults = {0};
    if (!options)
        options = &defaults;
    if ((options->width == 0) != (options->height == 0))
        return -EINVAL;

    bool vc4 = info->pipeline == CC_PIPELINE_RPI_UNICAM;
    bool pisp = info->pipeline == CC_PIPELINE_RPI_CFE;
    if (!vc4 && !pisp && options->profile != CC_VC4_PROFILE_AUTO)
        return -ENOTSUP;

    cc_capture_devices devices = {0};
    cc_capture *capture = NULL;
    cc_ipa *ipa = NULL;
    cc_still_configuration still = {.buffer_count = 4};
    int ret = 0;
    if (vc4 || pisp) {
        const cc_ipa_vc4_options tuning = {.profile = options->profile, .mains_hz = 50};
        ret = pisp ? cc_ipa_pisp_create(&tuning, &ipa) : cc_ipa_vc4_create(&tuning, &ipa);
        if (ret < 0)
            goto fail;
        ret = pisp ? cc_pisp_be_open_io(NULL, io, &devices.backend) :
                     cc_vc4_isp_open_io(NULL, io, &devices.vc4);
        if (ret < 0)
            goto fail;
    }
    ret = pisp ? cc_cfe_open_io(info, io, &devices.frontend) :
                 cc_camera_open_io(info, io, &devices.camera);
    if (ret < 0)
        goto fail;
    ret = vc4 || pisp ? configure_pi(&devices, options, &still) :
                       configure_native(devices.camera, options, &still);
    if (ret < 0)
        goto fail;
    ret = pisp ? cc_cfe_allocate_buffers(devices.frontend, &still.buffer_count) :
                 cc_camera_allocate_buffers(devices.camera, &still.buffer_count);
    if (ret < 0)
        goto fail;
    if (pisp)
        ret = cc_pisp_be_allocate_buffers(devices.backend, &still.buffer_count);
    if (vc4)
        ret = cc_vc4_isp_allocate_buffers(devices.vc4, &still.buffer_count);
    if (ret < 0)
        goto fail;
    ret = cc_capture_create(&devices, &capture, &still.buffer_count);
    if (ret < 0)
        goto fail;
    if (ipa) {
        ret = cc_capture_configure_sensor_defaults(capture);
        if (!ret)
            ret = cc_capture_set_ipa(capture, &ipa);
        if (ret < 0)
            goto fail;
    }
    *configuration = still;
    *out = capture;
    return 0;

fail:
    cc_capture_close(capture);
    cc_ipa_destroy(ipa);
    cc_capture_devices_close(&devices);
    return ret;
}

int cc_still_open(const cc_camera_info *camera, const cc_still_options *options,
                  cc_capture **capture, cc_still_configuration *configuration)
{
    return cc_still_open_io(camera, options, &cc_system_io, capture, configuration);
}

int cc_still_open_by_id(const char *id, const cc_still_options *options,
                        cc_capture **capture, cc_still_configuration *configuration)
{
    if (!capture)
        return -EINVAL;
    *capture = NULL;
    if (!configuration)
        return -EINVAL;
    cc_camera_info info;
    int ret = cc_camera_find_id_io(id, &cc_system_io, &info);
    return ret < 0 ? ret : cc_still_open(&info, options, capture, configuration);
}
