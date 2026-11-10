/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "monitor.h"
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#define CAMERA_LIMIT 4096U
#define EVENT_LIMIT 1024U
struct dependency { char path[CC_PATH_MAX]; struct cc_node_token token; };
struct record {
    cc_camera_info info;
    struct dependency *deps;
    size_t count,origin;
    bool revoked;
};
struct cc_camera_monitor {
    struct cc_io io;
    struct cc_monitor_backend backend;
    struct record *records;
    size_t count;
    cc_camera_event *events;
    size_t head,tail;
    uint64_t generation;
    /* Suppress a deleted node until it disappears, changes inode/device, or a
     * subsequent arrival/change notice establishes that it can be retried. */
    struct dependency *blocked;
    size_t num_blocked;
    unsigned settle,attempt;
    bool need_scan,lost;
};

static bool absent(int error)
{
    return error==-ENOENT || error==-ENODEV || error==-ENOTDIR;
}
static bool deferred(int error)
{
    return absent(error) || error==-EACCES || error==-EPERM || error==-EBUSY ||
        error==-EAGAIN || error==-ESTALE || error==-EIO;
}
static bool token_equal(struct cc_node_token a,struct cc_node_token b)
{
    return a.device==b.device && a.inode==b.inode;
}
static bool position_equal(const cc_camera_info *a,const cc_camera_info *b)
{
    return a->pipeline==b->pipeline && a->sensor_entity==b->sensor_entity &&
        !strcmp(a->video_path,b->video_path) && !strcmp(a->media_path,b->media_path);
}
static bool info_equal(const cc_camera_info *a,const cc_camera_info *b)
{
    return position_equal(a,b) && a->id_type==b->id_type && !strcmp(a->id,b->id) &&
        !strcmp(a->name,b->name) && !strcmp(a->driver,b->driver);
}
static unsigned strength(const cc_camera_info *info)
{
    if (!info->id[0]) return 0;
    return info->id_type==CC_CAMERA_ID_FIRMWARE || info->id_type==CC_CAMERA_ID_USB ? 2U:1U;
}
static void free_records(struct record *records,size_t count)
{
    if (!records) return;
    for (size_t i=0; i<count; ++i) free(records[i].deps);
    free(records);
}
static int clone_record(const struct record *old,struct record *out,size_t origin)
{
    struct dependency *deps=malloc(old->count*sizeof(*deps));
    if (!deps) return -ENOMEM;
    memcpy(deps,old->deps,old->count*sizeof(*deps));
    *out=*old; out->deps=deps; out->origin=origin; out->revoked=false;
    return 0;
}
static int add_dependency(cc_camera_monitor *m,struct record *record,const char *path)
{
    for (size_t i=0; i<record->count; ++i) if (!strcmp(record->deps[i].path,path)) return 0;
    struct cc_node_token token;
    int ret=m->backend.ops->probe(m->backend.ctx,path,&token);
    if (ret) return deferred(ret) ? -EAGAIN:ret;
    for (size_t i=0; i<m->num_blocked; ++i) if (!strcmp(path,m->blocked[i].path)) {
        if (token_equal(token,m->blocked[i].token)) return -EAGAIN;
        m->blocked[i]=m->blocked[--m->num_blocked]; break;
    }
    struct dependency *deps=realloc(record->deps,(record->count+1)*sizeof(*deps));
    if (!deps) return -ENOMEM;
    record->deps=deps;
    struct dependency *dep=&deps[record->count++];
    memset(dep,0,sizeof(*dep));
    memcpy(dep->path,path,strlen(path)+1); dep->token=token;
    return 0;
}
static int get_dependencies(cc_camera_monitor *m,struct record *record)
{
    int ret=add_dependency(m,record,record->info.video_path);
    if (ret || !record->info.media_path[0]) return ret;
    ret=add_dependency(m,record,record->info.media_path);
    if (ret) return ret;
    struct cc_media *media=NULL;
    ret=cc_media_open(&m->io,record->info.media_path,false,&media);
    if (ret) return deferred(ret) ? -EAGAIN:ret;
    /* Like the reference enumerator, wait for every media child node, including
     * subdevices that are not the selected image endpoint. */
    for (size_t i=0; i<media->num_entities; ++i) {
        const struct cc_entity *entity=&media->entities[i];
        if (!entity->desc.dev.major) continue;
        if (!entity->path[0]) { ret=-EAGAIN; break; }
        ret=add_dependency(m,record,entity->path);
        if (ret) break;
    }
    cc_media_close(media);
    return ret;
}
static int record_alive(cc_camera_monitor *m,const struct record *record,bool *pending)
{
    if (record->revoked) return 0;
    for (size_t i=0; i<record->count; ++i) {
        struct cc_node_token token;
        int ret=m->backend.ops->probe(m->backend.ctx,record->deps[i].path,&token);
        if (absent(ret)) return 0;
        if (ret) {
            if (!deferred(ret)) return ret;
            *pending=true; continue;
        }
        if (!token_equal(token,record->deps[i].token)) return 0;
    }
    return 1;
}

static int reconcile(cc_camera_monitor *m)
{
    for (size_t i=0; i<m->num_blocked;) {
        struct cc_node_token token;
        int status=m->backend.ops->probe(m->backend.ctx,m->blocked[i].path,&token);
        if (absent(status) || (!status && !token_equal(token,m->blocked[i].token)))
            m->blocked[i]=m->blocked[--m->num_blocked];
        else ++i;
    }
    cc_camera_info *fresh=NULL;
    size_t num=0,used=0;
    bool pending=false;
    int ret=cc_enumerate_state_io(&m->io,&fresh,&num,&pending);
    if (ret) goto fail_scan;
    struct record *next=calloc(num+m->count+1,sizeof(*next));
    bool *matched=calloc(m->count+1,sizeof(*matched));
    bool *alive=calloc(m->count+1,sizeof(*alive));
    bool *retained=calloc(m->count+1,sizeof(*retained));
    cc_camera_event *batch=calloc(EVENT_LIMIT,sizeof(*batch));
    cc_camera_event *queue=NULL;
    if (!next || !matched || !alive || !retained || !batch) { ret=-ENOMEM; goto done; }
    for (size_t i=0; i<m->count; ++i) {
        ret=record_alive(m,&m->records[i],&pending);
        if (ret<0) goto done;
        alive[i]=ret!=0;
    }
    for (size_t i=0; i<num; ++i) {
        size_t old=SIZE_MAX;
        for (size_t j=0; j<m->count; ++j) {
            if (matched[j]) continue;
            const cc_camera_info *info=&m->records[j].info;
            if ((fresh[i].id[0] && !strcmp(fresh[i].id,info->id)) ||
                (position_equal(&fresh[i],info) && (!fresh[i].id[0] || !info->id[0] || strength(&fresh[i])<strength(info)))) {
                old=j; break;
            }
        }
        struct record record={.info=fresh[i],.origin=SIZE_MAX};
        ret=get_dependencies(m,&record);
        if (ret) {
            free(record.deps);
            if (ret!=-EAGAIN) goto done;
            pending=true; continue;
        }
        if (old==SIZE_MAX) {
            /* A complete new identity at the same endpoint replaces the old
             * registration even if devtmpfs reused its character device. */
            for (size_t j=0; j<m->count; ++j)
                if (!matched[j] && position_equal(&record.info,&m->records[j].info)) { old=j; alive[j]=false; break; }
        }
        if (old!=SIZE_MAX) {
            matched[old]=true;
            for (size_t j=0; j<m->records[old].count; ++j) for (size_t k=0; k<record.count; ++k)
                if (!strcmp(m->records[old].deps[j].path,record.deps[k].path) &&
                    !token_equal(m->records[old].deps[j].token,record.deps[k].token)) alive[old]=false;
            if (alive[old]) {
                record.origin=old; retained[old]=true;
                if (strength(&record.info)<strength(&m->records[old].info)) {
                    record.info.id_type=m->records[old].info.id_type;
                    memcpy(record.info.id,m->records[old].info.id,sizeof(record.info.id));
                    pending=true;
                }
            }
        }
        next[used++]=record;
    }
    for (size_t i=0; i<m->count; ++i) if (!matched[i] && alive[i]) {
        ret=clone_record(&m->records[i],&next[used],i);
        if (ret) goto done;
        ++used; retained[i]=true; pending=true;
    }
    if (used>CAMERA_LIMIT) { ret=-E2BIG; goto done; }
    size_t events=0;
    /* Count every change even if the bounded batch can only hold RESYNC. */
    for (size_t i=0; i<m->count; ++i) if (!retained[i]) {
        if (events<EVENT_LIMIT) batch[events]=(cc_camera_event){.type=CC_CAMERA_REMOVED,.camera=m->records[i].info};
        ++events;
    }
    for (size_t i=0; i<used; ++i) {
        size_t old=next[i].origin;
        if (old==SIZE_MAX) {
            if (events<EVENT_LIMIT) batch[events]=(cc_camera_event){.type=CC_CAMERA_ADDED,.camera=next[i].info};
            ++events;
        } else if (!info_equal(&next[i].info,&m->records[old].info)) {
            if (events<EVENT_LIMIT) batch[events]=(cc_camera_event){.type=CC_CAMERA_CHANGED,.camera=next[i].info,.previous=m->records[old].info};
            ++events;
        }
    }
    size_t queued=m->tail-m->head;
    bool overflow=events>EVENT_LIMIT-queued;
    bool resync=m->lost || overflow;
    bool publish=events || resync || !m->generation;
    uint64_t generation=m->generation;
    if (publish) {
        if (generation==UINT64_MAX) { ret=-EOVERFLOW; goto done; }
        ++generation;
    }
    if (resync) {
        events=1; queued=0;
        batch[0]=(cc_camera_event){.type=CC_CAMERA_RESYNC,.flags=CC_CAMERA_EVENTS_LOST |
            (overflow ? CC_CAMERA_EVENT_QUEUE_OVERFLOW:0U)};
    }
    size_t total=queued+events;
    if (total) {
        queue=malloc(total*sizeof(*queue));
        if (!queue) { ret=-ENOMEM; goto done; }
        if (queued) memcpy(queue,m->events+m->head,queued*sizeof(*queue));
        for (size_t i=0; i<events; ++i) { batch[i].generation=generation; queue[queued+i]=batch[i]; }
    }
    uint32_t delay=0;
    if (pending || m->settle) delay=m->attempt>=6 ? 1000U:20U<<m->attempt;
    ret=m->backend.ops->retry(m->backend.ctx,delay);
    if (ret) goto done;
    ret=m->backend.ops->ready(m->backend.ctx,total!=0);
    if (ret) goto done;
    free_records(m->records,m->count); free(m->events);
    m->records=next; next=NULL; m->count=used;
    m->events=queue; queue=NULL; m->head=0; m->tail=total;
    m->generation=generation; m->lost=false; m->need_scan=false;
    if (m->settle) --m->settle;
    if (delay && m->attempt<6) ++m->attempt;
    ret=0;
done:
    free_records(next,used); free(fresh); free(matched); free(alive); free(retained); free(batch); free(queue);
    if (!ret) return 0;
    fresh=NULL;
fail_scan:
    free(fresh);
    m->need_scan=true;
    (void)m->backend.ops->retry(m->backend.ctx,1000);
    return ret;
}

int cc_camera_monitor_create_io(const struct cc_io *io,struct cc_monitor_backend *backend,cc_camera_monitor **out)
{
    if (!out) return -EINVAL;
    *out=NULL;
    if (!io || !io->ops || !backend || !backend->ops || backend->fd<0 ||
        !backend->ops->next || !backend->ops->ready || !backend->ops->retry || !backend->ops->probe ||
        !backend->ops->wait || !backend->ops->now || !backend->ops->close) return -EINVAL;
    cc_camera_monitor *m=calloc(1,sizeof(*m));
    if (!m) return -ENOMEM;
    m->io=*io; m->backend=*backend;
    int ret=reconcile(m);
    if (ret) { free(m->blocked); free(m); return ret; }
    *backend=(struct cc_monitor_backend){.fd=-1}; *out=m;
    return 0;
}
int cc_camera_monitor_open(cc_camera_monitor **out)
{
    if (!out) return -EINVAL;
    *out=NULL;
    struct cc_monitor_backend backend;
    int ret=cc_monitor_native_open(&backend);
    if (ret) return ret;
    ret=cc_camera_monitor_create_io(&cc_system_io,&backend,out);
    if (ret) backend.ops->close(backend.ctx);
    return ret;
}
void cc_camera_monitor_close(cc_camera_monitor *m)
{
    if (!m) return;
    m->backend.ops->close(m->backend.ctx);
    free_records(m->records,m->count); free(m->events); free(m->blocked); free(m);
}
int cc_camera_monitor_fd(const cc_camera_monitor *m) { return m ? m->backend.fd:-EINVAL; }
static int remove_notice(cc_camera_monitor *m,const char *path)
{
    const struct dependency *dep=NULL;
    for (size_t j=0; j<m->count; ++j) for (size_t k=0; k<m->records[j].count; ++k)
        if (!strcmp(m->records[j].deps[k].path,path)) {
            m->records[j].revoked=true; dep=&m->records[j].deps[k];
        }
    if (!dep) return 0;
    for (size_t i=0; i<m->num_blocked; ++i) if (!strcmp(m->blocked[i].path,path)) return 0;
    if (m->num_blocked>=65536) return -E2BIG;
    struct dependency *blocked=realloc(m->blocked,(m->num_blocked+1)*sizeof(*blocked));
    if (!blocked) return -ENOMEM;
    m->blocked=blocked; blocked[m->num_blocked++]=*dep;
    return 0;
}
int cc_camera_monitor_dispatch(cc_camera_monitor *m)
{
    if (!m) return -EINVAL;
    for (unsigned i=0; i<256; ++i) {
        struct cc_monitor_notice notice;
        int ret=m->backend.ops->next(m->backend.ctx,&notice);
        if (ret<0) return ret;
        if (!ret) break;
        if (ret!=1 || notice.type<CC_NOTICE_CHANGE || notice.type>CC_NOTICE_TIMER ||
            strnlen(notice.path,sizeof(notice.path))==sizeof(notice.path)) return -EPROTO;
        m->need_scan=true;
        if (notice.type!=CC_NOTICE_TIMER) { m->settle=8; m->attempt=0; }
        if (notice.type==CC_NOTICE_LOST) m->lost=true;
        if (notice.type==CC_NOTICE_REMOVE && notice.path[0]) {
            ret=remove_notice(m,notice.path);
            if (ret) { m->lost=true; (void)m->backend.ops->retry(m->backend.ctx,1000); return ret; }
        } else if (notice.type==CC_NOTICE_CHANGE && notice.path[0]) {
            for (size_t j=0; j<m->num_blocked; ++j) if (!strcmp(m->blocked[j].path,notice.path)) {
                m->blocked[j]=m->blocked[--m->num_blocked]; break;
            }
        }
    }
    return m->need_scan ? reconcile(m):0;
}
int cc_camera_monitor_refresh(cc_camera_monitor *m)
{
    if (!m) return -EINVAL;
    m->need_scan=true;
    return cc_camera_monitor_dispatch(m);
}
int cc_camera_monitor_snapshot(const cc_camera_monitor *m,cc_camera_info **out,size_t *count,uint64_t *generation)
{
    if (!m || !out || !count || !generation) return -EINVAL;
    cc_camera_info *list=NULL;
    if (m->count) {
        list=malloc(m->count*sizeof(*list));
        if (!list) return -ENOMEM;
        for (size_t i=0; i<m->count; ++i) list[i]=m->records[i].info;
    }
    *out=list; *count=m->count; *generation=m->generation;
    return 0;
}
int cc_camera_monitor_dequeue(cc_camera_monitor *m,cc_camera_event *out,int timeout)
{
    if (!m || !out || timeout<-1) return -EINVAL;
    uint64_t start=0;
    int ret;
    if (timeout>0 && (ret=m->backend.ops->now(m->backend.ctx,&start))) return ret;
    for (;;) {
        if (m->head!=m->tail) {
            if (m->head+1==m->tail) {
                ret=m->backend.ops->ready(m->backend.ctx,false);
                if (ret) return ret;
            }
            *out=m->events[m->head++]; return 0;
        }
        ret=cc_camera_monitor_dispatch(m);
        if (ret) return ret;
        if (m->head!=m->tail) continue;
        if (!timeout) return -EAGAIN;
        int remaining=-1;
        if (timeout>0) {
            uint64_t now;
            ret=m->backend.ops->now(m->backend.ctx,&now);
            if (ret) return ret;
            if (now<start) return -EIO;
            uint64_t elapsed=(now-start)/1000000U;
            if (elapsed>=(uint64_t)timeout) return -ETIMEDOUT;
            remaining=timeout-(int)elapsed;
        }
        ret=m->backend.ops->wait(m->backend.ctx,remaining);
        if (ret && ret!=-EINTR) return ret;
    }
}
