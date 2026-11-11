/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/ccamera.h>
#include <ccamera/stream.h>
#include <ccamera/vc4_isp.h>
#include <ccamera/cfe.h>
#include <ccamera/pisp_be.h>
#include <ccamera/capture.h>
#include <ccamera/ipa.h>
#include <ccamera/ipa_module.h>
#include <ccamera/ipa_manager.h>
#include <ccamera/ipa_proxy.h>
#include <ccamera/ipa_vimc.h>
#include <ccamera/ipc.h>
#include <ccamera/ipc_pipe.h>
#include <ccamera/process.h>
#include <ccamera/embedded.h>
#include <ccamera/vimc.h>
#include <ccamera/uvc.h>
#include <ccamera/monitor.h>
#include <ccamera/configuration.h>
#include <ccamera/linux/bcm2835-isp.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc == 2) {
        cc_ipa_proxy_options options = {.worker_path = argv[1]}; cc_ipa *installed_proxy = NULL;
        int ret = cc_ipa_vc4_create_proxy(NULL, &options, &installed_proxy);
        if (ret && ret != -ENOSYS && ret != -EPERM && ret != -EACCES) return 1;
        cc_ipa_destroy(installed_proxy);
    } else if (argc != 1) return 1;
    cc_process *process = NULL; cc_process_result process_result;
    if (cc_process_spawn(NULL, &process) != -EINVAL || process || cc_process_close(NULL) ||
        cc_process_pid(NULL) != -EINVAL || cc_process_fd(NULL) != -EINVAL ||
        cc_process_exec_fd(NULL) != -EINVAL || cc_process_wait_exec(NULL, 0) != -EINVAL ||
        cc_process_dispatch(NULL) != -EINVAL || cc_process_get_result(NULL, &process_result) != -EINVAL ||
        cc_process_wait(NULL, &process_result, 0) != -EINVAL || cc_process_signal(NULL, 0) != -EINVAL) return 1;
    cc_ipc *ipc = NULL; cc_ipc_packet *packet = NULL; cc_ipc_payload ipc_payload = {0};
    cc_ipc_message_header ipc_header = {0}; int remote = -1; short ipc_events = 0;
    cc_ipc_pipe *ipc_pipe = NULL;
    if (cc_ipc_pipe_open(NULL, -1, &ipc_pipe) != -EINVAL || ipc_pipe ||
        cc_ipc_pipe_fd(NULL) != -EINVAL || cc_ipc_pipe_peer_fd(NULL) != -EINVAL ||
        cc_ipc_pipe_events(NULL, &ipc_events) != -EINVAL || cc_ipc_pipe_pending(NULL) ||
        cc_ipc_pipe_dispatch(NULL) != -EINVAL || cc_ipc_pipe_send(NULL, &ipc_header, &ipc_payload) != -EINVAL ||
        cc_ipc_pipe_receive(NULL, &packet, 0) != -EINVAL || packet ||
        cc_ipc_pipe_call(NULL, &ipc_header, &ipc_payload, &packet, 0) != -EINVAL || packet) return 1;
    cc_ipc_pipe_close(NULL);
    if (cc_ipc_create(NULL, &remote) != -EINVAL || remote != -1 ||
        cc_ipc_bind(-1, &ipc) != -EINVAL || ipc || cc_ipc_fd(NULL) != -EINVAL ||
        cc_ipc_events(NULL, &ipc_events) != -EINVAL || cc_ipc_send(NULL, &ipc_payload) != -EINVAL ||
        cc_ipc_send_message(NULL, &ipc_header, &ipc_payload) != -EINVAL || cc_ipc_flush(NULL) != -EINVAL ||
        cc_ipc_receive(NULL, &packet) != -EINVAL || packet ||
        cc_ipc_packet_header(NULL, &ipc_header) != -EINVAL || cc_ipc_packet_take_fd(NULL, 0) != -EINVAL ||
        cc_ipc_packet_data(NULL, NULL) || cc_ipc_packet_fds(NULL, NULL)) return 1;
    cc_ipc_packet_destroy(NULL); cc_ipc_close(NULL);
    cc_ipa_manager *manager = NULL; cc_ipa *selected = NULL; size_t module_index = 0;
    cc_ipa_signature_status signature_status = CC_IPA_SIGNATURE_VALID;
    cc_ipa_execution_mode execution_mode = CC_IPA_EXECUTION_IN_PROCESS;
    if (cc_ipa_module_verify_signature(NULL, &signature_status) != -EINVAL ||
        cc_ipa_manager_verify_signature(NULL, 0, &signature_status) != -EINVAL ||
        signature_status != CC_IPA_SIGNATURE_VALID ||
        cc_ipa_module_create_auto(NULL, NULL, 0, NULL, &selected, &execution_mode) != -EINVAL || selected ||
        cc_ipa_manager_create_auto(NULL, "rpi/vc4", 1, 1, NULL, NULL, 0, NULL, &selected, &execution_mode) != -EINVAL || selected ||
        execution_mode != CC_IPA_EXECUTION_IN_PROCESS) return 1;
    uint32_t vimc_flags = 123; cc_ipa_vimc_event vimc_event;
    if (cc_ipa_vimc_create_proxy(NULL, NULL, &selected, &vimc_flags) != -EINVAL || selected || vimc_flags != 123 ||
        cc_ipa_vimc_map_buffers(NULL, NULL, 0) != -EINVAL || cc_ipa_vimc_unmap_buffers(NULL, NULL, 0) != -EINVAL ||
        cc_ipa_vimc_queue_request(NULL, 0, NULL, 0) != -EINVAL || cc_ipa_vimc_compute_params(NULL, 0, 0) != -EINVAL ||
        cc_ipa_vimc_dispatch(NULL) != -EINVAL || cc_ipa_vimc_dequeue_event(NULL, &vimc_event, 0) != -EINVAL) return 1;
    if (cc_ipa_module_create_proxy(NULL, NULL, 0, NULL, &selected) != -EINVAL || selected ||
        cc_ipa_reference_create_proxy(NULL, NULL, &selected) != -EINVAL || selected ||
        cc_ipa_vc4_create_proxy(NULL, NULL, NULL) != -EINVAL ||
        cc_ipa_manager_create_proxy(NULL, "rpi/vc4", 1, 1, NULL, NULL, 0, NULL, &selected) != -EINVAL || selected) return 1;
    if (cc_ipa_manager_open(NULL, NULL) != -EINVAL || cc_ipa_manager_count(manager) ||
        cc_ipa_manager_get_info(manager, 0) || cc_ipa_manager_get_path(manager, 0) ||
        cc_ipa_manager_select(manager, "rpi/vc4", 1, 1, NULL, &module_index) != -EINVAL ||
        cc_ipa_manager_create(manager, "rpi/vc4", 1, 1, NULL, NULL, 0, &selected) != -EINVAL || selected)
        return 1;
    cc_ipa_manager_close(manager);
    cc_ipa_manager_options build_only = {.flags = CC_IPA_MANAGER_BUILD_TREE};
    if (cc_ipa_manager_open(&build_only, &manager) || cc_ipa_manager_count(manager)) return 1;
    cc_ipa_manager_close(manager);
    cc_ipa_module *module = NULL; cc_ipa *loaded_ipa = NULL;
    const void *signature = NULL; size_t signature_size = 0;
    if (cc_ipa_module_open(NULL, &module) != -EINVAL || module ||
        cc_ipa_module_create(NULL, NULL, 0, &loaded_ipa) != -EINVAL || loaded_ipa ||
        cc_ipa_module_match(NULL, "rpi/vc4", 0, 0) != -EINVAL ||
        cc_ipa_module_get_signature(NULL, &signature, &signature_size) != -EINVAL ||
        cc_ipa_module_get_info(NULL) || cc_ipa_module_get_path(NULL)) return 1;
    cc_ipa_module_close(NULL);
    cc_pipeline_configuration pipeline={0};
    if (cc_pipeline_generate_configuration(NULL,NULL,0,&pipeline)!=-EINVAL ||
        cc_pipeline_validate_configuration(NULL,&pipeline)!=-EINVAL || cc_pipeline_configure(NULL,&pipeline)!=-EINVAL ||
        cc_camera_try_format_oriented(NULL,NULL,NULL)!=-EINVAL || cc_cfe_try_configuration_oriented(NULL,NULL,NULL)!=-EINVAL ||
        cc_cfe_get_configuration(NULL,NULL)!=-EINVAL || cc_vc4_isp_get_configuration(NULL,NULL)!=-EINVAL ||
        cc_pisp_be_get_configuration(NULL,NULL)!=-EINVAL) return 1;
    if (cc_cfe_try_configuration(NULL,NULL)!=-EINVAL ||
        cc_vc4_isp_try_configuration(NULL,NULL)!=-EINVAL ||
        cc_pisp_be_try_configuration(NULL,NULL)!=-EINVAL) return 1;
    cc_camera_event event;
    cc_camera_info *snapshot=NULL; size_t camera_count=0; uint64_t generation=0;
    if (cc_camera_monitor_open(NULL)!=-EINVAL || cc_camera_monitor_fd(NULL)!=-EINVAL ||
        cc_camera_monitor_dispatch(NULL)!=-EINVAL || cc_camera_monitor_refresh(NULL)!=-EINVAL ||
        cc_camera_monitor_snapshot(NULL,&snapshot,&camera_count,&generation)!=-EINVAL ||
        cc_camera_monitor_dequeue(NULL,&event,0)!=-EINVAL) return 1;
    cc_camera_monitor_close(NULL);
    cc_camera *identified=NULL; cc_cfe *identified_fe=NULL;
    if (cc_camera_open_by_id(NULL,&identified)!=-EINVAL || identified ||
        cc_cfe_open_by_id(NULL,&identified_fe)!=-EINVAL || identified_fe ||
        cc_cfe_get_info(NULL) || cc_capture_get_info(NULL)) return 1;
    enum cc_orientation orientation;
    enum cc_transform transform;
    if (cc_orientation_from_rotation(90,&orientation) || orientation!=CC_ORIENTATION_ROTATE_90 ||
        cc_orientation_transform(orientation,CC_ORIENTATION_ROTATE_0,&transform) || transform!=CC_TRANSFORM_ROTATE_90 ||
        cc_transform_inverse(transform,&transform) || transform!=CC_TRANSFORM_ROTATE_270 ||
        cc_orientation_apply(orientation,transform,&orientation) || orientation!=CC_ORIENTATION_ROTATE_0 ||
        cc_transform_compose(CC_TRANSFORM_HFLIP,CC_TRANSFORM_TRANSPOSE,&transform) || transform!=CC_TRANSFORM_ROTATE_270 ||
        cc_camera_try_orientation(NULL,&orientation,&transform)!=-EINVAL || cc_cfe_try_orientation(NULL,&orientation,&transform)!=-EINVAL ||
        cc_camera_get_orientation(NULL,&orientation)!=-EINVAL || cc_cfe_get_orientation(NULL,&orientation)!=-EINVAL ||
        cc_capture_get_orientation(NULL,&orientation)!=-EINVAL ||
        cc_camera_configure_oriented(NULL,NULL,&orientation)!=-EINVAL || cc_cfe_configure_oriented(NULL,NULL,&orientation)!=-EINVAL) return 1;
    cc_camera_properties properties;
    if (cc_camera_get_properties(NULL,&properties)!=-EINVAL ||
        cc_cfe_get_properties(NULL,&properties)!=-EINVAL ||
        cc_capture_get_properties(NULL,&properties)!=-EINVAL) return 1;
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
