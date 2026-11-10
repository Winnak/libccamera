/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/ipa.h>
#include <errno.h>
#include <float.h>
#include <stdbool.h>
#include <stdlib.h>
#include "ipa_module.h"

struct cc_ipa {
    cc_ipa_ops ops;
    void *context;
    bool configured, running;
    void *owner;
    void (*release_owner)(void *);
};

static int controls_valid(const cc_control *controls, size_t count, bool scalar)
{
    if (count > CC_IPA_MAX_CONTROLS || (count && !controls)) return -EINVAL;
    for (size_t i = 0; i < count; ++i) {
        if (!controls[i].id || (!controls[i].data != !controls[i].size) ||
            (scalar && (controls[i].data || controls[i].size))) return -EINVAL;
        for (size_t j = 0; j < i; ++j) if (controls[i].id == controls[j].id) return -EEXIST;
    }
    return 0;
}
static bool af_valid(const cc_af_status *s)
{
    return !(s->flags & ~CC_AF_LENS_VALID) && s->state>=CC_AF_IDLE && s->state<=CC_AF_FAILED &&
        s->pause_state>=CC_AF_RUNNING && s->pause_state<=CC_AF_PAUSED &&
        (!(s->flags & CC_AF_LENS_VALID) || (s->lens_position>=-DBL_MAX && s->lens_position<=DBL_MAX));
}
int cc_ipa_create(const cc_ipa_ops *ops, void *context, cc_ipa **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!ops) return -EINVAL;
    if (ops->abi_version != CC_IPA_ABI_VERSION || ops->size != sizeof(*ops)) return -EPROTONOSUPPORT;
    if (!ops->configure || !ops->start || !ops->stop || !ops->prepare || !ops->process ||
        ops->capabilities & ~(uint32_t)(CC_IPA_CAP_LENS|CC_IPA_CAP_SENSOR_TIMING|CC_IPA_CAP_SENSOR_GEOMETRY)) return -EINVAL;
    cc_ipa *ipa = calloc(1, sizeof(*ipa));
    if (!ipa) return -ENOMEM;
    ipa->ops = *ops; ipa->context = context; *out = ipa;
    return 0;
}
void cc_ipa_stop(cc_ipa *ipa)
{
    if (ipa && ipa->running) { ipa->running = false; ipa->ops.stop(ipa->context); }
}
void cc_ipa_destroy(cc_ipa *ipa)
{
    if (!ipa) return;
    cc_ipa_stop(ipa);
    if (ipa->ops.destroy) ipa->ops.destroy(ipa->context);
    if (ipa->release_owner) ipa->release_owner(ipa->owner);
    free(ipa);
}
void cc_ipa_set_owner(cc_ipa *ipa, void *owner, void (*release)(void *))
{
    ipa->owner = owner; ipa->release_owner = release;
}
void *cc_ipa_context(cc_ipa *ipa, int (*configure)(void *, const cc_ipa_configuration *))
{
    return ipa && ipa->ops.configure == configure ? ipa->context : NULL;
}
uint32_t cc_ipa_get_capabilities(const cc_ipa *ipa) { return ipa ? ipa->ops.capabilities:0; }
int cc_ipa_configure(cc_ipa *ipa, const cc_ipa_configuration *configuration)
{
    if (!ipa || !configuration || configuration->num_sensor_controls > CC_IPA_MAX_CONTROLS) return -EINVAL;
    const cc_embedded_configuration *embedded=&configuration->embedded;
    if (embedded->format.fourcc && (embedded->format.num_planes!=1 || !embedded->format.height ||
        !embedded->format.planes[0].stride || (embedded->bits_per_pixel!=8 && embedded->bits_per_pixel!=10 &&
        embedded->bits_per_pixel!=12 && embedded->bits_per_pixel!=14 && embedded->bits_per_pixel!=16) ||
        (uint64_t)embedded->format.planes[0].stride*embedded->format.height>embedded->format.planes[0].size)) return -EINVAL;
    if (ipa->running) return -EBUSY;
    int ret = controls_valid(configuration->initial_controls, configuration->num_sensor_controls, true);
    if (ret < 0) return ret;
    for (uint32_t i = 0; i < configuration->num_sensor_controls; ++i)
        if (configuration->sensor_controls[i].id != configuration->initial_controls[i].id) return -EINVAL;
    ipa->configured = false;
    ret = ipa->ops.configure(ipa->context, configuration);
    if (ret > 0) ret = -EPROTO;
    if (!ret) ipa->configured = true;
    return ret;
}
int cc_ipa_start(cc_ipa *ipa)
{
    if (!ipa) return -EINVAL;
    if (ipa->running) return -EBUSY;
    if (!ipa->configured) return -EPIPE;
    int ret = ipa->ops.start(ipa->context);
    if (ret > 0) ret = -EPROTO;
    if (!ret) ipa->running = true;
    return ret;
}
int cc_ipa_prepare(cc_ipa *ipa, const cc_ipa_request *request, cc_ipa_settings *settings)
{
    if (!ipa || !request || !settings) return -EINVAL;
    if (!ipa->running) return -EPIPE;
    int ret = controls_valid(request->sensor_controls, request->num_sensor_controls, true);
    if (ret < 0) return ret;
    if (request->af && cc_af_validate_controls(request->af)<0) return -EINVAL;
    cc_ipa_settings result = {0};
    ret = ipa->ops.prepare(ipa->context, request, &result);
    if (ret) return ret < 0 ? ret : -EPROTO;
    if (result.flags & ~(uint32_t)(CC_IPA_FRONTEND | CC_IPA_BACKEND | CC_IPA_LENS | CC_IPA_VC4_TUNING)) return -EPROTO;
    if (result.af.flags && (!(ipa->ops.capabilities & CC_IPA_CAP_LENS) || !af_valid(&result.af))) return -EPROTO;
    if ((result.flags & CC_IPA_LENS) && (!(ipa->ops.capabilities & CC_IPA_CAP_LENS) ||
        !(result.af.flags & CC_AF_LENS_VALID) || !af_valid(&result.af))) return -EPROTO;
    if (controls_valid(result.sensor_controls, result.num_sensor_controls, true) < 0 ||
        controls_valid(result.isp_controls, result.num_isp_controls, false) < 0) return -EPROTO;
    if ((result.flags & CC_IPA_VC4_TUNING) && cc_vc4_validate_tuning(&result.vc4)<0) return -EPROTO;
    *settings = result;
    return 0;
}
int cc_ipa_process(cc_ipa *ipa, const cc_ipa_frame *frame, cc_ipa_metadata *metadata)
{
    if (!ipa || !frame || !metadata || frame->sensor.count > CC_SENSOR_MAX_CONTROLS ||
        (frame->statistics_size && !frame->statistics_data) || (frame->embedded_size && !frame->embedded_data)) return -EINVAL;
    if (!ipa->running) return -EPIPE;
    if (controls_valid(frame->isp_controls, frame->num_isp_controls, false) < 0 ||
        controls_valid(frame->request_sensor_controls,frame->num_request_sensor_controls,true)<0 ||
        controls_valid(frame->settings_sensor_controls,frame->num_settings_sensor_controls,true)<0 ||
        (frame->vc4 && cc_vc4_validate_tuning(frame->vc4)<0)) return -EINVAL;
    cc_ipa_metadata result = {0};
    int ret = ipa->ops.process(ipa->context, frame, &result);
    if (ret) return ret < 0 ? ret : -EPROTO;
    if (result.flags & ~(uint32_t)(CC_IPA_METADATA_AE | CC_IPA_METADATA_AWB | CC_IPA_AE_CONVERGED |
        CC_IPA_AWB_CONVERGED | CC_IPA_METADATA_AF | CC_IPA_METADATA_SENSOR | CC_IPA_METADATA_PDAF | CC_IPA_METADATA_HDR | CC_IPA_METADATA_VC4 | CC_IPA_EXPOSURE_LIMIT)) return -EPROTO;
    if (((result.flags & CC_IPA_AE_CONVERGED) && !(result.flags & CC_IPA_METADATA_AE)) ||
        ((result.flags & CC_IPA_AWB_CONVERGED) && !(result.flags & CC_IPA_METADATA_AWB))) return -EPROTO;
    if ((result.flags & CC_IPA_METADATA_AE) && (!result.exposure_ns || !(result.analogue_gain >= 1) ||
        !(result.analogue_gain <= DBL_MAX) || !(result.luminance >= 0 && result.luminance <= 1))) return -EPROTO;
    if ((result.flags & CC_IPA_METADATA_AWB) && (!(result.red_gain > 0 && result.red_gain <= DBL_MAX) ||
        !(result.blue_gain > 0 && result.blue_gain <= DBL_MAX))) return -EPROTO;
    if ((result.flags & CC_IPA_METADATA_AF) && !af_valid(&result.af)) return -EPROTO;
    if ((result.flags&CC_IPA_METADATA_SENSOR) && (!result.sensor_status.line_length_pck || !result.sensor_status.frame_length_lines ||
        !(result.sensor_status.analogue_gain>=1 && result.sensor_status.analogue_gain<=DBL_MAX) ||
        result.sensor_status.flags&~CC_SENSOR_STATUS_TEMPERATURE ||
        ((result.sensor_status.flags&CC_SENSOR_STATUS_TEMPERATURE) &&
         (result.sensor_status.temperature_c < -20 || result.sensor_status.temperature_c>80)))) return -EPROTO;
    if ((result.flags&CC_IPA_METADATA_HDR) && (!result.hdr.counted || result.hdr.average>=65536)) return -EPROTO;
    if ((result.flags & CC_IPA_METADATA_VC4) && (!(result.colour_temperature>=2000 && result.colour_temperature<=13000) ||
        !(result.lux>=0 && result.lux<=DBL_MAX) || !result.frame_duration_ns)) return -EPROTO;
    *metadata = result;
    return 0;
}
