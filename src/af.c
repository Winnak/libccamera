/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* C17 port of the RPi autofocus controller from the supplied libcamera reference.
 * Copyright (C) 2022-2023 Raspberry Pi Ltd. See LICENSES/BSD-2-Clause.txt.
 * JSON/geometry/PWL containers are replaced by explicit bounded C structures. */
#include <ccamera/af.h>
#include <ccamera/linux/bcm2835-isp.h>
#include <ccamera/linux/pisp/pisp_fe_statistics.h>
#include <errno.h>
#include <float.h>
#include <limits.h>
#include <linux/videodev2.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

enum scan_state { IDLE, TRIGGER, PDAF, COARSE1, COARSE2, FINE, SETTLE };
struct record { double focus, contrast, phase, confidence; };
struct weights { uint32_t rows, cols, sum; uint16_t values[CC_AF_MAX_REGIONS]; };
struct cc_af {
    cc_af_tuning tuning;
    cc_control_info lens;
    cc_rectangle region, windows[CC_AF_MAX_WINDOWS];
    uint32_t num_windows, range, speed;
    enum cc_af_mode mode;
    enum scan_state scan;
    enum cc_af_state report;
    bool configured, paused, initted, ir, use_windows;
    double target, smooth, previous_contrast, old_contrast, previous_rgb[3], old_rgb[3], previous_phase;
    uint32_t skip, step, drop, same_sign, scene_change, max_index, count, capacity;
    double max_contrast, min_contrast, scan_step;
    struct record *records;
    struct weights phase_weights, contrast_weights, colour_weights;
    uint64_t native_focus[64];
    cc_af_colour native_colour[CC_AF_MAX_REGIONS];
};
static double minimum(double a, double b) { return a<b ? a:b; }
static double maximum(double a, double b) { return a>b ? a:b; }
static double absolute(double x) { return x<0 ? -x:x; }
static double clamp(double x, double a, double b) { return maximum(a,minimum(x,b)); }
static bool finite(double x) { return x>=-DBL_MAX && x<=DBL_MAX; }
static uint32_t increment(uint32_t x) { return x==UINT32_MAX ? x:x+1; }
static const cc_af_speed_tuning *speed(const cc_af *a) { return &a->tuning.speeds[a->speed]; }
static const cc_af_range_tuning *range(const cc_af *a) { return &a->tuning.ranges[a->range]; }
static void invalidate(cc_af *a) { a->phase_weights.sum=a->contrast_weights.sum=a->colour_weights.sum=0; }
static void idle(cc_af *a) { a->scan=IDLE; a->report=CC_AF_IDLE; a->count=0; }

void cc_af_reference_tuning(cc_af_tuning *t)
{
    if (!t) return;
    *t=(cc_af_tuning){.confidence_epsilon=8,.confidence_threshold=16,.confidence_clip=512,.skip_frames=5,
        .num_map_points=2,.map={{0,445},{15,925}}};
    for (size_t i=0;i<3;++i) t->ranges[i]=(cc_af_range_tuning){0,12,1};
    for (size_t i=0;i<2;++i) t->speeds[i]=(cc_af_speed_tuning){1,0.25,0.75,0.75,10,-0.02,0.125,2,20,6,4};
}
static int tuning_valid(const cc_af_tuning *t, uint32_t *capacity)
{
    if (!t || t->num_map_points<2 || t->num_map_points>CC_AF_MAX_MAP || !t->confidence_epsilon ||
        t->confidence_epsilon>65535 || t->confidence_threshold>t->confidence_clip ||
        t->confidence_clip>65535 || t->skip_frames>1000000 || t->check_for_ir>1) return -EINVAL;
    for (uint32_t i=0;i<t->num_map_points;++i) {
        const cc_af_map_point *p=&t->map[i];
        if (!(p->dioptres>=-1000 && p->dioptres<=1000 && p->code>=INT32_MIN && p->code<=INT32_MAX) ||
            (i && (p->dioptres<=t->map[i-1].dioptres || p->code<=t->map[i-1].code))) return -EINVAL;
    }
    uint32_t needed=4;
    for (size_t i=0;i<3;++i) {
        const cc_af_range_tuning *r=&t->ranges[i];
        if (!(r->minimum>=t->map[0].dioptres && r->maximum<=t->map[t->num_map_points-1].dioptres &&
              r->maximum>=r->minimum && r->default_position>=r->minimum && r->default_position<=r->maximum)) return -EINVAL;
        for (size_t j=0;j<2;++j) {
            const cc_af_speed_tuning *s=&t->speeds[j];
            if (!(s->step_coarse>0 && s->step_coarse<=2000 && s->step_fine>=0 && s->step_fine<=2000 &&
                s->contrast_ratio>0 && s->contrast_ratio<1 && s->retrigger_ratio>0 && s->retrigger_ratio<1 &&
                finite(s->pdaf_gain) && absolute(s->pdaf_gain)<=1000000 && s->pdaf_squelch>0 && s->pdaf_squelch<=2000 &&
                s->max_slew>0 && s->max_slew<=2000) || !s->retrigger_delay || s->retrigger_delay>1000000 ||
                s->pdaf_frames>1000000 || s->dropout_frames>1000000 || s->step_frames>1000000) return -EINVAL;
            double steps=(r->maximum-r->minimum)/s->step_coarse;
            if (!(steps<=4090)) return -E2BIG;
            uint32_t n=(uint32_t)steps+6;
            if (n>needed) needed=n;
        }
    }
    *capacity=needed; return 0;
}
int cc_af_create(const cc_af_tuning *t, cc_af **out)
{
    if (!out) return -EINVAL;
    *out=NULL; uint32_t capacity;
    int ret=tuning_valid(t,&capacity); if (ret<0) return ret;
    cc_af *a=calloc(1,sizeof(*a)); if (!a) return -ENOMEM;
    a->records=calloc(capacity,sizeof(*a->records));
    if (!a->records) { free(a); return -ENOMEM; }
    a->tuning=*t; a->capacity=capacity; *out=a; return 0;
}
void cc_af_destroy(cc_af *a) { if (a) { free(a->records); free(a); } }
int cc_af_configure(cc_af *a, const cc_rectangle *r, const cc_control_info *lens)
{
    if (!a || !r || !lens || !r->width || !r->height || r->width>65535 || r->height>65535 ||
        lens->id!=V4L2_CID_FOCUS_ABSOLUTE || lens->type!=V4L2_CTRL_TYPE_INTEGER || !lens->step ||
        lens->minimum<INT32_MIN || lens->maximum>INT32_MAX || lens->minimum>lens->maximum ||
        lens->elems>1 || lens->flags & (V4L2_CTRL_FLAG_READ_ONLY|V4L2_CTRL_FLAG_DISABLED|V4L2_CTRL_FLAG_INACTIVE|
                                      V4L2_CTRL_FLAG_HAS_PAYLOAD|V4L2_CTRL_FLAG_MODIFY_LAYOUT)) return -EINVAL;
    if (a->tuning.map[0].code<(double)lens->minimum ||
        a->tuning.map[a->tuning.num_map_points-1].code>(double)lens->maximum) return -ERANGE;
    cc_af_tuning tuning=a->tuning; struct record *records=a->records; uint32_t capacity=a->capacity;
    memset(a,0,sizeof(*a)); a->tuning=tuning; a->records=records; a->capacity=capacity;
    a->region=*r; a->lens=*lens; a->configured=true; a->target=a->smooth=-1; a->skip=tuning.skip_frames;
    return 0;
}
static double interpolate(const cc_af *a, double value, bool inverse)
{
    uint32_t n=a->tuning.num_map_points, i=0;
    const cc_af_map_point *p=a->tuning.map;
    double first=inverse ? p[0].code:p[0].dioptres, last=inverse ? p[n-1].code:p[n-1].dioptres;
    value=clamp(value,first,last);
    while (i+2<n && value>(inverse ? p[i+1].code:p[i+1].dioptres)) ++i;
    double x0=inverse ? p[i].code:p[i].dioptres, x1=inverse ? p[i+1].code:p[i+1].dioptres;
    double y0=inverse ? p[i].dioptres:p[i].code, y1=inverse ? p[i+1].dioptres:p[i+1].code;
    return y0+(value-x0)*(y1-y0)/(x1-x0);
}
int cc_af_lens_position(const cc_af *a, int32_t code, double *out)
{
    if (!a || !out) return -EINVAL;
    if (!a->configured) return -EPIPE;
    *out=interpolate(a,code,true); return 0;
}
static void update_lens(cc_af *a)
{
    if (a->scan>=PDAF) a->target=clamp(a->target,range(a)->minimum,range(a)->maximum);
    if (a->initted) a->smooth=clamp(a->target,a->smooth-speed(a)->max_slew,a->smooth+speed(a)->max_slew);
    else { a->smooth=a->target; a->initted=true; a->skip=a->tuning.skip_frames; }
}
static void start_scan(cc_af *a)
{
    if (!a->initted || a->mode!=CC_AF_CONTINUOUS || a->smooth<=range(a)->minimum+2*speed(a)->step_coarse) {
        a->target=range(a)->minimum; a->scan_step=speed(a)->step_coarse; a->scan=COARSE2;
    } else if (a->smooth>=range(a)->maximum-2*speed(a)->step_coarse) {
        a->target=range(a)->maximum; a->scan_step=-speed(a)->step_coarse; a->scan=COARSE2;
    } else { a->scan_step=-speed(a)->step_coarse; a->scan=COARSE1; }
    a->max_contrast=0; a->min_contrast=1e9; a->max_index=a->count=0;
    a->step=speed(a)->step_frames; a->report=CC_AF_SCANNING;
}
static void start_af(cc_af *a)
{
    if (speed(a)->pdaf_gain!=0 && speed(a)->dropout_frames && (a->mode==CC_AF_CONTINUOUS || speed(a)->pdaf_frames)) {
        if (!a->initted) { a->target=range(a)->default_position; update_lens(a); }
        a->step=a->mode==CC_AF_CONTINUOUS ? 0:speed(a)->pdaf_frames;
        a->scan=PDAF; a->count=a->drop=a->scene_change=0; a->old_contrast=0; a->report=CC_AF_SCANNING;
    } else { start_scan(a); update_lens(a); }
}
int cc_af_validate_controls(const cc_af_controls *c)
{
    if (!c || c->flags & ~511U || ((c->flags & CC_AF_TRIGGER) && (c->flags & CC_AF_CANCEL)) ||
        ((c->flags & CC_AF_SET_MODE) && (c->mode<CC_AF_MANUAL || c->mode>CC_AF_CONTINUOUS)) ||
        ((c->flags & CC_AF_SET_RANGE) && (c->range<CC_AF_NORMAL || c->range>CC_AF_FULL)) ||
        ((c->flags & CC_AF_SET_SPEED) && (c->speed<CC_AF_SPEED_NORMAL || c->speed>CC_AF_SPEED_FAST)) ||
        ((c->flags & CC_AF_SET_PAUSE) && (c->pause<CC_AF_PAUSE_IMMEDIATE || c->pause>CC_AF_RESUME)) ||
        ((c->flags & CC_AF_SET_METERING) && c->use_windows>1) ||
        ((c->flags & CC_AF_SET_LENS) && !finite(c->lens_position)) ||
        ((c->flags & CC_AF_SET_WINDOWS) && c->num_windows>CC_AF_MAX_WINDOWS)) return -EINVAL;
    if (c->flags & CC_AF_SET_WINDOWS) for (uint32_t i=0;i<c->num_windows;++i)
        if (!c->windows[i].width || !c->windows[i].height) return -EINVAL;
    return 0;
}
int cc_af_set_controls(cc_af *a, const cc_af_controls *c)
{
    if (!a) return -EINVAL;
    int ret=cc_af_validate_controls(c); if (ret<0) return ret;
    if (!a->configured) return -EPIPE;
    if ((c->flags & CC_AF_SET_MODE) && a->mode!=c->mode) {
        a->mode=c->mode; a->paused=false;
        if (a->mode==CC_AF_CONTINUOUS) a->scan=TRIGGER;
        else if (a->mode!=CC_AF_AUTO || a->scan<COARSE1) idle(a);
    }
    if (c->flags & CC_AF_SET_RANGE) a->range=(uint32_t)c->range;
    if (c->flags & CC_AF_SET_SPEED) {
        if (a->scan==PDAF && a->tuning.speeds[c->speed].pdaf_frames>speed(a)->pdaf_frames) {
            uint32_t extra=a->tuning.speeds[c->speed].pdaf_frames-speed(a)->pdaf_frames;
            a->step=a->step>UINT32_MAX-extra ? UINT32_MAX:a->step+extra;
        }
        a->speed=(uint32_t)c->speed;
    }
    if (c->flags & CC_AF_SET_METERING) { a->use_windows=c->use_windows!=0; invalidate(a); }
    if (c->flags & CC_AF_SET_WINDOWS) {
        a->num_windows=c->num_windows; memcpy(a->windows,c->windows,c->num_windows*sizeof(*a->windows)); invalidate(a);
    }
    if ((c->flags & CC_AF_SET_LENS) && a->mode==CC_AF_MANUAL) {
        a->target=clamp(c->lens_position,a->tuning.map[0].dioptres,a->tuning.map[a->tuning.num_map_points-1].dioptres);
        update_lens(a);
    }
    if ((c->flags & CC_AF_CANCEL) && a->mode==CC_AF_AUTO) idle(a);
    if ((c->flags & CC_AF_TRIGGER) && a->mode==CC_AF_AUTO && a->scan==IDLE) a->scan=TRIGGER;
    if ((c->flags & CC_AF_SET_PAUSE) && a->mode==CC_AF_CONTINUOUS) {
        if (c->pause==CC_AF_RESUME && a->paused) {
            a->paused=false; if (a->scan<COARSE1) a->scan=TRIGGER;
        } else if (c->pause!=CC_AF_RESUME && !a->paused) {
            a->paused=true;
            if (c->pause==CC_AF_PAUSE_IMMEDIATE || a->scan<COARSE1) { a->scan=IDLE; a->count=0; }
        }
    }
    return 0;
}
static int grid_valid(uint32_t rows, uint32_t cols, const void *data)
{
    return rows>CC_AF_MAX_REGIONS || cols>CC_AF_MAX_REGIONS || (uint64_t)rows*cols>CC_AF_MAX_REGIONS ||
        (!rows != !cols) || (rows && !data) ? -EINVAL:0;
}
static void weights(cc_af *a, struct weights *w, uint32_t rows, uint32_t cols)
{
    if (w->rows==rows && w->cols==cols && w->sum) return;
    w->rows=rows; w->cols=cols; w->sum=0; memset(w->values,0,sizeof(w->values));
    if (rows && cols && a->use_windows && a->region.width>=cols && a->region.height>=rows) {
        uint32_t max_cell=46080U/(CC_AF_MAX_WINDOWS*rows*cols), h=a->region.height/rows, width=a->region.width/cols;
        uint64_t area=(uint64_t)h*width;
        for (uint32_t i=0;i<a->num_windows;++i) for (uint32_t r=0;r<rows;++r) {
            int64_t y0=(int64_t)a->region.y+h*r, y1=y0+h;
            if (y0<a->windows[i].y) y0=a->windows[i].y;
            if (y1>(int64_t)a->windows[i].y+a->windows[i].height) y1=(int64_t)a->windows[i].y+a->windows[i].height;
            if (y0>=y1) continue;
            for (uint32_t c=0;c<cols;++c) {
                int64_t x0=(int64_t)a->region.x+width*c, x1=x0+width;
                if (x0<a->windows[i].x) x0=a->windows[i].x;
                if (x1>(int64_t)a->windows[i].x+a->windows[i].width) x1=(int64_t)a->windows[i].x+a->windows[i].width;
                if (x0>=x1) continue;
                uint32_t value=(uint32_t)((max_cell*(uint64_t)(y1-y0)*(uint64_t)(x1-x0)+area-1)/area);
                w->values[r*cols+c]=(uint16_t)(w->values[r*cols+c]+value); w->sum+=value;
            }
        }
    }
    if (!w->sum) for (uint32_t r=rows/3;r<rows-rows/3;++r) for (uint32_t c=cols/4;c<cols-cols/4;++c) {
        w->values[r*cols+c]=1; ++w->sum;
    }
}
static void phase(cc_af *a, const cc_af_pdaf_grid *g, double *out_phase, double *confidence)
{
    *out_phase=*confidence=0; if (!g) return;
    weights(a,&a->phase_weights,g->rows,g->cols);
    uint64_t wc=0; int64_t wcp=0;
    for (uint32_t i=0;i<g->rows*g->cols;++i) {
        uint32_t c=g->regions[i].confidence, w=a->phase_weights.values[i];
        if (c<a->tuning.confidence_threshold) continue;
        if (c>a->tuning.confidence_clip) c=a->tuning.confidence_clip;
        c-=a->tuning.confidence_threshold>>1;
        wc+=(uint64_t)w*c; wcp+=(int64_t)((uint64_t)w*c)*g->regions[i].phase;
    }
    if (a->phase_weights.sum && wc>=a->phase_weights.sum) {
        *out_phase=(double)wcp/(double)wc; *confidence=(double)wc/a->phase_weights.sum;
    }
}
static void do_pdaf(cc_af *a, double p, double confidence)
{
    p*=speed(a)->pdaf_gain;
    if (a->mode==CC_AF_CONTINUOUS) {
        p*=confidence/(confidence+a->tuning.confidence_epsilon);
        if (absolute(p)<speed(a)->pdaf_squelch) { double v=p/speed(a)->pdaf_squelch; p*=v*v; }
    } else if (a->step>=speed(a)->step_frames) {
        if (absolute(p)<speed(a)->pdaf_squelch) a->step=speed(a)->step_frames;
    } else p*=a->step/speed(a)->step_frames;
    if (p < -speed(a)->max_slew) {
        p=-speed(a)->max_slew; a->report=a->target<=range(a)->minimum ? CC_AF_FAILED:CC_AF_SCANNING;
    } else if (p>speed(a)->max_slew) {
        p=speed(a)->max_slew; a->report=a->target>=range(a)->maximum ? CC_AF_FAILED:CC_AF_SCANNING;
    } else a->report=CC_AF_FOCUSED;
    a->target=a->smooth+p;
}
static bool early_phase(cc_af *a, double p)
{
    if (!a->count || a->records[a->count-1].confidence<a->tuning.confidence_threshold) return false;
    const struct record *last=&a->records[a->count-1];
    if ((a->target-last->focus)*(p-last->phase)*speed(a)->pdaf_gain<0) {
        double t=p/(p-last->phase);
        if ((t>=-2.5 || a->mode==CC_AF_CONTINUOUS) && t<=3) {
            a->target+=maximum(t,-2.5)*(last->focus-a->target); return true;
        }
    }
    return false;
}
static double find_peak(const cc_af *a, uint32_t i)
{
    double f=a->records[i].focus;
    if (a->count>=3) {
        if (!i) ++i; else if (i+1>=a->count) --i;
        double ax=a->records[i-1].focus-a->records[i].focus, ay=a->records[i-1].contrast-a->records[i].contrast;
        double cx=a->records[i+1].focus-a->records[i].focus, cy=a->records[i+1].contrast-a->records[i].contrast;
        double denominator=2*(ay*cx-cy*ax);
        if (absolute(denominator)>=1.0/64 && denominator*ax>0)
            f=a->records[i].focus+clamp((ay*cx*cx-cy*ax*ax)/denominator,minimum(ax,cx),maximum(ax,cx));
    }
    return f;
}
static void do_scan(cc_af *a, double contrast, double p, double confidence)
{
    if (!a->count || contrast>a->max_contrast) {
        a->max_contrast=contrast; a->max_index=a->count;
        if (a->scan!=FINE) memcpy(a->old_rgb,a->previous_rgb,sizeof(a->old_rgb));
    }
    if (contrast<a->min_contrast) a->min_contrast=contrast;
    if (a->count>=a->capacity) { a->scan=IDLE; a->report=CC_AF_FAILED; return; }
    a->records[a->count++]=(struct record){a->target,contrast,p,confidence};
    if ((a->scan_step>=0 && a->target>=range(a)->maximum) || (a->scan_step<=0 && a->target<=range(a)->minimum) ||
        (a->scan==FINE && a->count>=3) || contrast<speed(a)->contrast_ratio*a->max_contrast) {
        double peak=find_peak(a,a->max_index);
        if (a->scan==COARSE1 && a->records[0].contrast>=speed(a)->contrast_ratio*a->max_contrast) {
            a->scan_step=-a->scan_step; a->scan=COARSE2;
        } else if (a->scan==FINE || speed(a)->step_fine<=0) { a->target=peak; a->scan=SETTLE; }
        else if (a->scan_step>=0) {
            a->target=minimum(peak+speed(a)->step_fine,range(a)->maximum); a->scan_step=-speed(a)->step_fine; a->scan=FINE;
        } else {
            a->target=maximum(peak-speed(a)->step_fine,range(a)->minimum); a->scan_step=speed(a)->step_fine; a->scan=FINE;
        }
        a->count=0;
    } else a->target+=a->scan_step;
    a->step=a->target==a->smooth ? 0:speed(a)->step_frames;
}
static void do_af(cc_af *a, double contrast, double p, double confidence)
{
    if (a->skip) { --a->skip; return; }
    a->same_sign=p*a->previous_phase<=0 ? 0:increment(a->same_sign); a->previous_phase=p;
    if (a->mode==CC_AF_MANUAL || (a->paused && a->scan<COARSE1)) return;
    if (a->scan==PDAF) {
        if (confidence>=a->tuning.confidence_epsilon) {
            if (a->mode==CC_AF_AUTO || a->same_sign>=3) do_pdaf(a,p,confidence);
            if (a->step) --a->step; else if (a->mode!=CC_AF_CONTINUOUS) a->scan=IDLE;
            a->old_contrast=contrast; memcpy(a->old_rgb,a->previous_rgb,sizeof(a->old_rgb)); a->scene_change=a->drop=0;
            return;
        }
        a->drop=increment(a->drop);
        if (a->drop<speed(a)->dropout_frames) return;
        if (a->mode!=CC_AF_CONTINUOUS) { start_scan(a); return; }
    }
    if (a->scan<COARSE1 && a->mode==CC_AF_CONTINUOUS) {
        bool changed=contrast+1<speed(a)->retrigger_ratio*a->old_contrast || a->old_contrast+1<speed(a)->retrigger_ratio*contrast;
        for (size_t i=0;i<3;++i) changed=changed || a->previous_rgb[i]+1<speed(a)->retrigger_ratio*a->old_rgb[i] ||
            a->old_rgb[i]+1<speed(a)->retrigger_ratio*a->previous_rgb[i];
        if (changed) { a->old_contrast=contrast; memcpy(a->old_rgb,a->previous_rgb,sizeof(a->old_rgb)); a->scene_change=1; }
        else if (a->scene_change) a->scene_change=increment(a->scene_change);
        if (a->scene_change>=speed(a)->retrigger_delay) start_scan(a);
    } else if (a->scan>=COARSE1 && a->smooth==a->target) {
        if (a->step) --a->step;
        else if (a->scan==SETTLE) {
            a->report=a->previous_contrast>=speed(a)->contrast_ratio*a->max_contrast &&
                a->min_contrast<=speed(a)->contrast_ratio*a->max_contrast ? CC_AF_FOCUSED:CC_AF_FAILED;
            a->scan=a->mode==CC_AF_CONTINUOUS && !a->paused && speed(a)->dropout_frames ? PDAF:IDLE;
            a->drop=a->scene_change=0; a->old_contrast=maximum(a->max_contrast,a->previous_contrast); a->count=0;
        } else if (confidence>=a->tuning.confidence_threshold && early_phase(a,p)) {
            memcpy(a->old_rgb,a->previous_rgb,sizeof(a->old_rgb)); a->scan=SETTLE;
            a->step=a->mode==CC_AF_CONTINUOUS ? 0:speed(a)->step_frames;
        } else do_scan(a,contrast,p,confidence);
    }
}
int cc_af_prepare(cc_af *a, const cc_af_pdaf_grid *pdaf, cc_af_status *out)
{
    if (!a || !out || (pdaf && grid_valid(pdaf->rows,pdaf->cols,pdaf->regions)<0)) return -EINVAL;
    if (!a->configured) return -EPIPE;
    if (a->scan==TRIGGER) start_af(a);
    if (a->initted) { double p,c; phase(a,pdaf,&p,&c); do_af(a,a->previous_contrast,p,a->ir ? 0:c); update_lens(a); }
    cc_af_status s={.state=a->mode==CC_AF_AUTO && a->scan!=IDLE ? CC_AF_SCANNING:
        a->mode==CC_AF_MANUAL ? CC_AF_IDLE:a->report,
        .pause_state=a->paused ? a->scan==IDLE ? CC_AF_PAUSED:CC_AF_PAUSING:CC_AF_RUNNING};
    if (a->initted) {
        int64_t code=(int32_t)interpolate(a,a->smooth,false);
        code=a->lens.minimum+(int64_t)((uint64_t)(code-a->lens.minimum)/a->lens.step*a->lens.step);
        s.flags=CC_AF_LENS_VALID; s.lens_setting=(int32_t)code; s.lens_position=interpolate(a,(double)code,true);
    }
    *out=s; return 0;
}
int cc_af_process(cc_af *a, const cc_af_statistics *s)
{
    if (!a || !s || grid_valid(s->focus_rows,s->focus_cols,s->focus)<0 ||
        grid_valid(s->colour_rows,s->colour_cols,s->colour)<0) return -EINVAL;
    if (!a->configured) return -EPIPE;
    weights(a,&a->contrast_weights,s->focus_rows,s->focus_cols);
    weights(a,&a->colour_weights,s->colour_rows,s->colour_cols);
    uint64_t sum=0, rgb[3]={0}, sw=1, grey=0, all=0;
    for (uint32_t i=0;i<s->focus_rows*s->focus_cols;++i) {
        uint64_t w=a->contrast_weights.values[i];
        if (w && s->focus[i]>(UINT64_MAX-sum)/w) return -EOVERFLOW;
        sum+=w*s->focus[i];
    }
    for (uint32_t i=0;i<s->colour_rows*s->colour_cols;++i) {
        const cc_af_colour *c=&s->colour[i]; uint64_t w=a->colour_weights.values[i];
        const uint64_t values[]={c->red,c->green,c->blue};
        for (size_t j=0;j<3;++j) {
            if (values[j]>UINT64_MAX/5 || (w && values[j]>(UINT64_MAX/5-rgb[j])/w)) return -EOVERFLOW;
            rgb[j]+=w*values[j];
        }
        sw+=w;
        uint64_t r=c->red,g=c->green,b=c->blue;
        if (4*r<5*b && 4*b<5*r && 4*r<5*g && 4*g<5*r && 4*b<5*g && 4*g<5*b) grey+=c->counted;
        all+=c->counted;
    }
    a->previous_contrast=a->contrast_weights.sum ? (double)sum/a->contrast_weights.sum:0;
    for (size_t i=0;i<3;++i) a->previous_rgb[i]=(double)rgb[i]/(double)sw;
    a->ir=a->tuning.check_for_ir && 2*grey>all && 4*rgb[0]<5*rgb[2] && 4*rgb[2]<5*rgb[0] &&
        4*rgb[0]<5*rgb[1] && 4*rgb[1]<5*rgb[0] && 4*rgb[2]<5*rgb[1] && 4*rgb[1]<5*rgb[2];
    return 0;
}
int cc_af_process_native(cc_af *a, uint32_t fourcc, const void *data, size_t size)
{
    if (!a || !data) return -EINVAL;
    if (!a->configured) return -EPIPE;
    cc_af_statistics s={.focus=a->native_focus,.colour=a->native_colour};
    if (fourcc==CC_FOURCC('B','S','T','A')) {
        struct bcm2835_isp_stats b;
        if (size<sizeof(b)) return -EMSGSIZE;
        memcpy(&b,data,sizeof(b)); if (b.size && (b.size<sizeof(b) || b.size>size)) return -EPROTO;
        s.focus_rows=3; s.focus_cols=4; s.colour_rows=12; s.colour_cols=16;
        for (size_t i=0;i<FOCUS_REGIONS;++i) a->native_focus[i]=b.focus_stats[i].contrast_val[1][1]/1000;
        for (size_t i=0;i<AWB_REGIONS;++i) a->native_colour[i]=(cc_af_colour){
            b.awb_stats[i].r_sum,b.awb_stats[i].g_sum,b.awb_stats[i].b_sum,b.awb_stats[i].counted};
    } else if (fourcc==CC_FOURCC('R','P','F','S')) {
        struct pisp_statistics p; if (size<sizeof(p)) return -EMSGSIZE; memcpy(&p,data,sizeof(p));
        s.focus_rows=s.focus_cols=8; s.colour_rows=s.colour_cols=32;
        for (size_t i=0;i<PISP_CDAF_STATS_NUM_FOMS;++i) a->native_focus[i]=p.cdaf.foms[i]>>20;
        for (size_t i=0;i<PISP_AWB_STATS_NUM_ZONES;++i) a->native_colour[i]=(cc_af_colour){
            p.awb.zones[i].R_sum,p.awb.zones[i].G_sum,p.awb.zones[i].B_sum,p.awb.zones[i].counted};
    } else return -ENOTSUP;
    return cc_af_process(a,&s);
}
