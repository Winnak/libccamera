/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/ccamera.h>
#include <ccamera/stream.h>
#include <ccamera/vc4_isp.h>
#include <ccamera/cfe.h>
#include <ccamera/pisp_be.h>
#include <ccamera/capture.h>
#include <ccamera/ipa.h>
#include <ccamera/embedded.h>
#include <ccamera/vimc.h>
#include <ccamera/uvc.h>
#include <ccamera/linux/bcm2835-isp.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    const uint32_t mode_code=0x300f;
    cc_sensor_mode_request mode_request={.mbus_codes=&mode_code,.num_mbus_codes=1,.width=640,.height=480};
    cc_sensor_format sensor_format;
    if (cc_camera_select_sensor_mode(NULL,&mode_request,&sensor_format)!=-EINVAL ||
        cc_cfe_select_sensor_mode(NULL,&mode_request,&sensor_format)!=-EINVAL) return 1;
    cc_rpi_sensor_mode_request rpi_mode_request={.mbus_codes=&mode_code,.num_mbus_codes=1,
        .width=640,.height=480,.bit_depth=12};
    if (cc_camera_select_rpi_sensor_mode(NULL,&rpi_mode_request,&sensor_format)!=-EINVAL ||
        cc_cfe_select_rpi_sensor_mode(NULL,&rpi_mode_request,&sensor_format)!=-EINVAL) return 1;
    cc_fraction interval={1,30}; cc_frame_interval_range intervals;
    if (cc_stream_get_frame_interval(NULL,&interval)!=-EINVAL || cc_stream_set_frame_interval(NULL,&interval)!=-EINVAL ||
        cc_stream_enum_frame_interval(NULL,1,1,1,0,&intervals)!=-EINVAL ||
        cc_camera_get_frame_interval(NULL,&interval)!=-EINVAL || cc_camera_set_frame_interval(NULL,&interval)!=-EINVAL ||
        cc_camera_enum_frame_interval(NULL,1,1,1,0,&intervals)!=-EINVAL ||
        cc_camera_enum_sensor_frame_interval(NULL,1,1,1,0,&interval)!=-EINVAL ||
        cc_cfe_get_frame_interval(NULL,&interval)!=-EINVAL || cc_cfe_set_frame_interval(NULL,&interval)!=-EINVAL ||
        cc_cfe_enum_sensor_frame_interval(NULL,1,1,1,0,&interval)!=-EINVAL ||
        cc_capture_get_frame_interval(NULL,&interval)!=-EINVAL) return 1;
    cc_uvc_profile uvc_profile={0}; cc_uvc_range uvc_range;
    cc_uvc_controls uvc={0}; cc_control_menu menu;
    size_t uvc_count=0;
    if (cc_uvc_controls_to_native(&uvc_profile,&uvc,NULL,&uvc_count) || uvc_count ||
        cc_uvc_get_control_range(&uvc_profile,CC_UVC_CONTROL_GAMMA,&uvc_range)!=-ENOENT ||
        cc_camera_get_uvc_profile(NULL,&uvc_profile)!=-EINVAL || cc_camera_set_uvc_controls(NULL,&uvc)!=-EINVAL ||
        cc_request_set_uvc_controls(NULL,&uvc)!=-EINVAL || cc_camera_query_menu(NULL,1,0,&menu)!=-EINVAL ||
        cc_capture_query_menu(NULL,1,0,&menu)!=-EINVAL || cc_cfe_query_menu(NULL,1,0,&menu)!=-EINVAL ||
        cc_video_query_menu(NULL,1,0,&menu)!=-EINVAL || cc_vc4_isp_query_menu(NULL,1,0,&menu)!=-EINVAL) return 1;
    cc_vimc_controls vimc={.flags=CC_VIMC_BRIGHTNESS,.brightness=0};
    cc_control native; size_t native_count=1;
    if (cc_vimc_controls_to_native(&vimc,&native,&native_count) || native.value!=128 ||
        cc_camera_set_vimc_controls(NULL,&vimc)!=-EINVAL ||
        cc_request_set_vimc_controls(NULL,&vimc)!=-EINVAL) return 1;
    if (cc_request_set_buffer(NULL,CC_CAPTURE_RAW,NULL,0)!=-EINVAL ||
        cc_cfe_allocate_buffers_with_memory(NULL,CC_MEMORY_DMABUF,NULL)!=-EINVAL ||
        cc_cfe_allocate_outputs(NULL,NULL,NULL)!=-EINVAL ||
        cc_cfe_import_output(NULL,1,0,NULL,0)!=-EINVAL ||
        cc_cfe_import_buffer(NULL,0,NULL,0)!=-EINVAL ||
        cc_vc4_isp_allocate_buffers_with_memory(NULL,NULL,NULL)!=-EINVAL ||
        cc_vc4_isp_import_output(NULL,0,0,NULL,0)!=-EINVAL ||
        cc_pisp_be_allocate_buffers_with_memory(NULL,NULL,NULL)!=-EINVAL ||
        cc_pisp_be_import_output(NULL,0,0,NULL,0)!=-EINVAL) return 1;
    if (cc_pisp_be_submit_with_auxiliary(NULL,0,NULL,0,NULL,NULL,NULL,0)!=-EINVAL ||
        cc_pisp_be_export_port_plane(NULL,CC_PISP_BE_TDN_OUTPUT,0,0)!=-EINVAL) return 1;
    int fence_timeout;
    if (cc_request_set_acquire_fence(NULL,CC_CAPTURE_RAW,-1)!=-EINVAL ||
        cc_request_take_acquire_fence(NULL,CC_CAPTURE_RAW)!=-EINVAL ||
        cc_request_set_fence_timeout(NULL,300)!=-EINVAL ||
        cc_capture_get_timeout(NULL,&fence_timeout)!=-EINVAL) return 1;
    cc_smia_layout layout={8,0,1};
    const uint8_t embedded[]={0x0a,0xaa,0x12,0xa5,0x34,0x5a,0x56};
    cc_smia_register report={0x1234,0};
    if (cc_smia_parse(&layout,embedded,sizeof(embedded),&report,1) || report.value!=0x56) return 1;
    cc_af_tuning af_tuning; cc_af_reference_tuning(&af_tuning);
    cc_af *af=NULL;
    if (cc_af_create(&af_tuning,&af)) return 1;
    cc_af_destroy(af);
    if (strcmp(cc_version(), "0.1.0"))
        return 1;
    cc_ipa_reference_tuning tuning={.flags=CC_IPA_REFERENCE_AWB,.target_luminance=0.18,
        .exposure_speed=0.5,.white_balance_speed=0.5,.min_colour_gain=0.5,.max_colour_gain=4};
    cc_ipa *ipa=NULL;
    if (cc_ipa_reference_create(&tuning,&ipa) || !ipa)
        return 1;
    cc_ipa_destroy(ipa);
    cc_request *request = NULL;
    if (cc_request_create(123, &request) || cc_request_get_status(request) != CC_REQUEST_IDLE ||
        cc_request_set_tuning(request, NULL, NULL) || cc_request_set_isp_controls(request, NULL, 0) ||
        cc_request_set_sensor_controls(request, NULL, 0) ||
        cc_request_get_result(request) || cc_request_destroy(request))
        return 1;
    cc_capture_devices empty = {0};
    cc_capture *capture = NULL;
    uint32_t slots = 0;
    if (cc_capture_create(&empty, &capture, &slots) != -EINVAL || capture)
        return 1;
    cc_sensor_control_delay delay = {.id = 1, .frames = 2, .flags = CC_SENSOR_CONTROL_PRIORITY};
    if (cc_capture_configure_sensor_delays(NULL, &delay, 1) != -EINVAL ||
        cc_capture_schedule_sensor_controls(NULL, 2, NULL, 0, 0) != -EINVAL)
        return 1;
    const cc_sensor_model *sensor;
    uint32_t gain_code;
    double gain;
    if (cc_sensor_lookup("imx219 10-0010", &sensor) || !cc_sensor_get_properties(sensor) ||
        cc_sensor_gain_code(sensor, 2, &gain_code) || gain_code != 128 ||
        cc_sensor_gain(sensor, gain_code, &gain) || gain != 2 ||
        cc_capture_configure_sensor_defaults(NULL) != -EINVAL)
        return 1;
    cc_sensor_timing timing = {.pixel_rate=96000000,.width=1920,.height=1080,.hblank=280,.vblank=45};
    uint64_t duration;
    if (cc_sensor_frame_duration(&timing, &duration) || duration != 25781250)
        return 1;
    cc_sensor_timing_limits limits={96000000,1920,1080,280,280,1,45,1000,1,1,1};
    cc_sensor_exposure_plan exposure_plan;
    uint64_t maximum_duration;
    cc_sensor_mode_description mode={1920,1080,1,1,25781250}; cc_sensor_mode_policy policy;
    if (cc_sensor_timing_bounds(&limits,&duration,&maximum_duration) ||
        cc_sensor_plan_exposure(sensor,&limits,30000000,0,0,&exposure_plan) ||
        cc_sensor_get_mode_policy(sensor,&mode,&policy) || policy.sensitivity!=1 ||
        cc_camera_get_sensor_timing_limits(NULL,&limits)!=-EINVAL ||
        cc_cfe_get_sensor_timing_limits(NULL,&limits)!=-EINVAL ||
        cc_capture_get_sensor_timing_limits(NULL,&limits)!=-EINVAL) return 1;
    cc_sensor_streams sensor_streams;
    if (cc_camera_get_sensor_streams(NULL,&sensor_streams)!=-EINVAL ||
        cc_cfe_get_sensor_streams(NULL,&sensor_streams)!=-EINVAL ||
        cc_capture_get_sensor_streams(NULL,&sensor_streams)!=-EINVAL) return 1;
    cc_embedded_configuration embedded_configuration;
    if (CC_IPA_ABI_VERSION!=5 || cc_camera_configure_embedded(NULL,&embedded_configuration)!=-EINVAL ||
        cc_cfe_configure_embedded(NULL,&embedded_configuration)!=-EINVAL ||
        cc_capture_get_embedded_configuration(NULL,&embedded_configuration)!=-EINVAL ||
        cc_camera_export_embedded_plane(NULL,0,0)!=-EINVAL) return 1;
    cc_video_device *device = NULL;
    if (cc_video_open("/this-camera-device-does-not-exist", &device) != -ENOENT || device)
        return 1;
    if (cc_stream_fd(NULL) != -EINVAL)
        return 1;
    cc_vc4_isp *isp = NULL;
    if (cc_vc4_isp_open("/this-camera-device-does-not-exist", &isp) != -ENOENT || isp)
        return 1;
    cc_pisp_be *be = NULL;
    if (cc_pisp_be_open("/this-camera-device-does-not-exist", &be) != -ENOENT || be)
        return 1;
    if (sizeof(struct bcm2835_isp_stats) < sizeof(struct bcm2835_isp_stats_hist))
        return 1;
    cc_camera_info cfe_info = { .pipeline = CC_PIPELINE_RPI_CFE };
    strcpy(cfe_info.media_path, "/this-camera-device-does-not-exist");
    strcpy(cfe_info.video_path, "/this-camera-device-does-not-exist");
    cc_cfe *cfe = NULL;
    if (cc_cfe_open(&cfe_info, &cfe) != -ENOENT || cfe)
        return 1;
    cc_cfe_configuration configuration = {
        .sensor = { .fourcc = CC_FOURCC('p', 'R', 'A', 'A'), .width = 640, .height = 480 },
        .image = { .fourcc = CC_FOURCC('R', 'G', '1', '6'), .width = 640, .height = 480,
                   .num_planes = 1, .planes = { {1280, 1280 * 480} } }
    };
    struct pisp_fe_config parameters;
    if (cc_cfe_build_parameters(&configuration, NULL, &parameters) ||
        !(parameters.global.enables & PISP_FE_ENABLE_OUTPUT0))
        return 1;
    cc_format rgb = { .fourcc = CC_FOURCC('R', 'G', 'B', '3'), .width = 640, .height = 480,
        .num_planes = 1, .planes = { {1920, 1920 * 480} } };
    struct pisp_be_tiles_config be_parameters;
    if (cc_pisp_be_build_parameters(&configuration.image, &rgb, NULL, &be_parameters) || !be_parameters.num_tiles)
        return 1;
    cc_pisp_be_configuration dual = { .input = configuration.image, .output = rgb,
        .output1 = { .fourcc = CC_FOURCC('N', 'V', '2', '1'), .width = 640, .height = 480,
                     .num_planes = 1, .planes = { {640, 640 * 480 * 3 / 2} } } };
    if (cc_pisp_be_build_configuration(&dual, NULL, &be_parameters) ||
        !(be_parameters.config.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT1) ||
        cc_pisp_be_export_output_plane(NULL, 1, 0, 0) != -EINVAL)
        return 1;
    dual.crop[0] = (cc_rectangle) {0, 0, 640, 360};
    dual.output.width = 320; dual.output.height = 180;
    dual.transform[0] = PISP_BE_TRANSFORM_ROT180;
    if (cc_pisp_be_build_configuration(&dual, NULL, &be_parameters) ||
        !(be_parameters.config.global.rgb_enables & PISP_BE_RGB_ENABLE_RESAMPLE0))
        return 1;
    struct pisp_be_config manual = {0};
    manual.global.bayer_enables = PISP_BE_BAYER_ENABLE_CAC | PISP_BE_BAYER_ENABLE_SDN;
    manual.global.rgb_enables = PISP_BE_RGB_ENABLE_CCM;
    manual.ccm.offsets[0] = 134217728;
    dual.hardware_version = CC_PISP_BE_VERSION_BCM2712_D0;
    if (cc_pisp_be_build_configuration(&dual, &manual, &be_parameters) ||
        !be_parameters.config.cac.grid_step_x || be_parameters.config.ccm.offsets[0] != 134217728)
        return 1;
    cc_camera_info *cameras = NULL;
    size_t count = 0;
    int ret = cc_enumerate(&cameras, &count);
    cc_camera_list_free(cameras);
    if (ret < 0)
        return 1;
    printf("libccamera %s: %zu accessible cameras\n", cc_version(), count);
    return 0;
}
