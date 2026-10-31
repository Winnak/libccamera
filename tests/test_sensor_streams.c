/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "sensor_stream_driver.h"

static void test_discovery_and_routing(void)
{
    for (unsigned legacy=0;legacy<2;++legacy) {
        struct fake f; initialize(&f,false); f.legacy=legacy!=0;
        cc_sensor_streams streams; OK(cc_camera_get_sensor_streams(&f.camera,&streams));
        CHECK(streams.flags==(CC_SENSOR_STREAMS_MULTIPLEXED|CC_SENSOR_STREAMS_EMBEDDED));
        CHECK(streams.image.pad==2 && streams.image.stream==7 && !streams.image.flags && streams.image.width==640);
        CHECK(streams.embedded.pad==2 && streams.embedded.stream==42 && streams.embedded.flags==CC_SENSOR_STREAM_ACTIVE);
        CHECK(streams.embedded.mbus_code==0x8002 && streams.embedded.width==800 && streams.embedded.height==4);
        cc_sensor_mode mode; OK(cc_camera_enum_sensor_mode(&f.camera,0,0,&mode));
        CHECK(f.size_stream==7 && mode.sizes.max_width==640);
        /* TRY and ACTIVE may have different route IDs; TRY cannot change the
         * selected active stream cached by the configured camera. */
        f.image[0]=19; f.routes[0][0].source_stream=19;
        cc_format format=raw_format(); OK(cc_camera_try_format(&f.camera,&format));
        CHECK(f.received_stream==19 && f.camera.sensor_stream==7 && !f.stream.configured);
        OK(cc_camera_get_sensor_streams(&f.camera,&streams)); CHECK(!streams.image.flags && streams.embedded.flags);
        format=raw_format(); OK(cc_camera_configure(&f.camera,&format));
        CHECK(f.received_stream==7 && f.stream.configured && format.width==640 && format.height==480);
        OK(cc_camera_get_sensor_streams(&f.camera,&streams));
        CHECK(streams.image.flags==CC_SENSOR_STREAM_ACTIVE && !streams.embedded.flags);
        CHECK(f.routes[1][2].flags==CC_SD_ROUTE_ACTIVE && f.sensor_sets==2 && f.receiver_sets==2);
        CHECK(f.current_gets && (legacy ? f.old_gets>0:!f.old_gets));
        CHECK(f.current_sets && (legacy ? f.old_sets>0:!f.old_sets));
    }
    struct fake f; initialize(&f,false); f.reverse_set=true;
    cc_format format=raw_format(); OK(cc_camera_configure(&f.camera,&format));
    cc_sensor_streams streams; OK(cc_camera_get_sensor_streams(&f.camera,&streams)); CHECK(streams.image.stream==7);
    /* Discovery is live, including a changed driver route ID. */
    initialize(&f,false); f.image[1]=31; f.routes[1][0].source_stream=31;
    OK(cc_camera_get_sensor_streams(&f.camera,&streams)); CHECK(streams.image.stream==31);
    cc_sensor_mode mode; OK(cc_camera_enum_sensor_mode(&f.camera,0,0,&mode)); CHECK(f.size_stream==31);
}
static void expect_discovery_error(struct fake *f, int error)
{
    cc_sensor_streams streams; memset(&streams,0xe7,sizeof(streams)); cc_sensor_streams saved=streams;
    CHECK(cc_camera_get_sensor_streams(&f->camera,&streams)==-error);
    CHECK(!memcmp(&streams,&saved,sizeof(saved)));
}
static void test_rejection(void)
{
    for (unsigned legacy=0;legacy<2;++legacy) {
        struct fake f;
#define RESET() do { initialize(&f,false); f.legacy=legacy!=0; } while (0)
        RESET(); f.duplicate_image=true; expect_discovery_error(&f,ENOTUNIQ);
        RESET(); f.duplicate_embedded=true; expect_discovery_error(&f,ENOTUNIQ);
        RESET(); f.routes[1][2]=f.routes[1][0]; expect_discovery_error(&f,EPROTO);
        RESET(); f.routes[1][0].sink_pad=4; expect_discovery_error(&f,EPROTO);
        RESET(); f.routes[1][0].source_pad=0; expect_discovery_error(&f,EPROTO);
        RESET(); f.bad_format_stream=true; expect_discovery_error(&f,EPROTO);
        RESET(); f.bad_code_stream=true; expect_discovery_error(&f,EPROTO);
        RESET(); f.mixed_codes=true; expect_discovery_error(&f,EPROTO);
        RESET(); f.bad_get_count=true; expect_discovery_error(&f,E2BIG);
        RESET(); f.empty_routes=true; expect_discovery_error(&f,ENODATA);
        RESET(); f.bad_get_pointer=true; expect_discovery_error(&f,EPROTO);
        RESET(); f.endless_codes=true; expect_discovery_error(&f,E2BIG);
        RESET(); f.count[1]=1; f.routes[1][0]=f.routes[1][1]; expect_discovery_error(&f,ENOTSUP);
        RESET(); f.fail_request=legacy ? CC_SD_G_ROUTING_LEGACY:CC_SD_G_ROUTING; expect_discovery_error(&f,EIO);
        RESET(); f.fail_request=VIDIOC_SUBDEV_ENUM_MBUS_CODE; expect_discovery_error(&f,EIO);
        RESET(); f.fail_request=VIDIOC_SUBDEV_G_FMT; expect_discovery_error(&f,EIO);
        RESET(); f.bad_set=true; cc_format format=raw_format(),saved=format;
        CHECK(cc_camera_configure(&f.camera,&format)==-EPROTO && !f.stream.configured && !memcmp(&format,&saved,sizeof(saved)));
        RESET(); f.bad_receiver=true; format=raw_format(); CHECK(cc_camera_configure(&f.camera,&format)==-EPROTO);
        RESET(); f.streams[1]=false; format=raw_format(); CHECK(cc_camera_configure(&f.camera,&format)==-ENOTSUP);
        RESET(); f.fail_request=legacy ? CC_SD_S_ROUTING_LEGACY:CC_SD_S_ROUTING; f.fail_fd=SENSOR;
        format=raw_format(); CHECK(cc_camera_configure(&f.camera,&format)==-EIO);
#undef RESET
    }
    CHECK(cc_camera_get_sensor_streams(NULL,NULL)==-EINVAL);
    CHECK(cc_cfe_get_sensor_streams(NULL,NULL)==-EINVAL && cc_capture_get_sensor_streams(NULL,NULL)==-EINVAL);
    struct fake f; initialize(&f,false); f.camera.media=NULL;
    expect_discovery_error(&f,ENOTSUP);
}
static void test_legacy_pads(void)
{
    struct fake f; initialize(&f,true);
    cc_sensor_streams streams; OK(cc_camera_get_sensor_streams(&f.camera,&streams));
    CHECK(streams.flags==CC_SENSOR_STREAMS_EMBEDDED && streams.image.pad==0 && !streams.image.stream);
    CHECK(streams.embedded.pad==1 && !streams.embedded.stream && streams.embedded.mbus_code==0x7001);
    f.streams[1]=false; cc_format format=raw_format(); OK(cc_camera_configure(&f.camera,&format));
    CHECK(!f.current_sets && !f.old_sets && !f.receiver_routes);
    f.metadata=false; OK(cc_camera_get_sensor_streams(&f.camera,&streams)); CHECK(!streams.flags);
    f.metadata=true; f.duplicate_embedded=true; expect_discovery_error(&f,ENOTUNIQ);
}
static void test_open_discovery(void)
{
    cc_camera_info info={.pipeline=CC_PIPELINE_RPI_CFE,.sensor_entity=1};
    strcpy(info.video_path,"/video"); strcpy(info.media_path,"/media");
    for (unsigned legacy=0;legacy<2;++legacy) {
        struct fake f; initialize(&f,false); f.legacy=legacy!=0;
        cc_camera *camera=NULL; OK(cc_camera_open_io(&info,&f.camera.io,&camera));
        CHECK(camera && camera->sensor_pad==2 && camera->sensor_stream==7);
        cc_format format=raw_format(); OK(cc_camera_configure(camera,&format));
        CHECK(f.received_stream==7); cc_camera_close(camera); CHECK(f.opened==f.closed);
    }
    for (unsigned bad=0;bad<3;++bad) {
        struct fake f; initialize(&f,false);
        if (bad==0) f.bad_get_count=true;
        if (bad==1) f.duplicate_image=true;
        if (bad==2) f.reject_client=true;
        cc_camera *camera=(cc_camera *)(uintptr_t)1;
        int ret=cc_camera_open_io(&info,&f.camera.io,&camera);
        CHECK(ret==(bad==0 ? -E2BIG:bad==1 ? -ENOTUNIQ:-EPROTO) && !camera && f.opened==f.closed);
    }
}
static void test_intervals(void)
{
    for (unsigned fixed=0;fixed<2;++fixed) for (unsigned modern=0;modern<2;++modern) {
        struct fake f; initialize(&f,fixed!=0); f.interval_modern=modern!=0;
        cc_format format=raw_format(); OK(cc_camera_configure(&f.camera,&format));
        cc_fraction interval; OK(cc_camera_get_frame_interval(&f.camera,&interval));
        CHECK(interval.numerator==1001 && interval.denominator==30000 && f.interval_stream==(fixed ? 0U:7U));
        interval=(cc_fraction){1,25}; OK(cc_camera_set_frame_interval(&f.camera,&interval));
        CHECK(interval.numerator==1001 && interval.denominator==24000);
        OK(cc_camera_enum_sensor_frame_interval(&f.camera,MEDIA_BUS_FMT_SRGGB10_1X10,640,480,0,&interval));
        CHECK(interval.numerator==1 && interval.denominator==30);
        cc_fraction saved=interval;
        CHECK(cc_camera_enum_sensor_frame_interval(&f.camera,MEDIA_BUS_FMT_SRGGB10_1X10,640,480,1,&interval)==-ENOENT);
        f.bad_interval=true; CHECK(cc_camera_get_frame_interval(&f.camera,&interval)==-EPROTO);
        CHECK(!memcmp(&interval,&saved,sizeof(interval)));
        CHECK(cc_camera_enum_sensor_frame_interval(&f.camera,MEDIA_BUS_FMT_SRGGB10_1X10,640,480,0,&interval)==-EPROTO);
        f.stream.allocated=true; CHECK(cc_camera_set_frame_interval(&f.camera,&interval)==-EBUSY);
    }
}
int main(void)
{
    test_discovery_and_routing(); test_rejection(); test_legacy_pads(); test_open_discovery(); test_intervals();
    puts("Live sensor stream discovery, arbitrary image IDs, TRY/ACTIVE routing and bounded protocol checks passed");
    return 0;
}
