#include "branchguard.h"
#include <math.h>
#include <string.h>

static const int scan[BG_ROWS] = {112,100,88,77,67,58,50,43,37,32,28,25};
static float clamp(float x, float low, float high) {
    return fminf(high, fmaxf(low, x));
}
int bg_pixel_row(int row) { return row >= 0 && row < BG_ROWS ? scan[row] : -1; }
float bg_distance(int row) {
    return row >= 0 && row < BG_ROWS ? 18.0f/(scan[row]-12.0f) : 0.0f;
}
void bg_init(BGState *state) { if (state) memset(state,0,sizeof(*state)); }

/* A local contrast threshold avoids treating a whole dark row as a line.
   Broad runs (e.g. a transverse stripe) are intentionally rejected. */
void bg_extract(const uint8_t *image, int stride, BGObservation *out) {
    int r;
    if (!out) return;
    memset(out,0,sizeof(*out));
    if (!image || stride < BG_WIDTH) return;
    for (r=0;r<BG_ROWS;r++) {
        const uint8_t *line=image+scan[r]*stride;
        int x,lo=255,hi=0,start=-1;
        float threshold;
        for (x=0;x<BG_WIDTH;x++) { if (line[x]<lo) lo=line[x]; if (line[x]>hi) hi=line[x]; }
        if (hi-lo<38) continue;
        threshold=lo+0.44f*(hi-lo);
        for (x=0;x<=BG_WIDTH;x++) {
            if (x<BG_WIDTH && line[x]<threshold) { if (start<0) start=x; }
            else if (start>=0) {
                int width=x-start;
                if (width>=2 && width<=30 && out->rows[r].count<BG_CANDIDATES) {
                    BGPoint *p=&out->rows[r].points[out->rows[r].count++];
                    p->y=(94.0f-0.5f*(start+x-1))*bg_distance(r)/80.0f;
                    p->quality=clamp((hi-lo)/160.0f,0.1f,1.0f);
                }
                start=-1;
            }
        }
    }
}

float bg_path_at(const BGPath *path, float s) {
    int i,last=-1;
    if (!path || !path->mask) return 0.0f;
    for (i=0;i<BG_ROWS;i++) if (path->mask & (1u<<i)) {
        if (bg_distance(i)>=s) {
            if (last<0) return path->y[i];
            return path->y[last]+(path->y[i]-path->y[last])*
                (s-bg_distance(last))/(bg_distance(i)-bg_distance(last));
        }
        last=i;
    }
    return path->y[last];
}

static float separation(const BGPath *a, const BGPath *b) {
    static const float probes[3]={0.45f,0.8f,1.15f};
    float d=0; int j;
    for (j=0;j<3;j++) d+=fabsf(bg_path_at(a,probes[j])-bg_path_at(b,probes[j]));
    return d/3.0f;
}
static int last_row(const BGPath *p, int before) {
    int i; for (i=before-1;i>=0;i--) if (p->mask & (1u<<i)) return i;
    return -1;
}
static void insert(BGPath beam[BG_BEAM], int *n, BGPath item) {
    int k=*n;
    if (k==BG_BEAM && item.cost>=beam[k-1].cost) return;
    if (k<BG_BEAM) (*n)++; else k--;
    while (k>0 && beam[k-1].cost>item.cost) { beam[k]=beam[k-1]; k--; }
    beam[k]=item;
}
int bg_search(const BGObservation *obs, BGPath paths[BG_PATHS]) {
    BGPath beam[BG_BEAM],next[BG_BEAM];
    int n=1,r,i,j,used=0;
    if (!obs || !paths) return 0;
    memset(beam,0,sizeof(beam));
    for (r=0;r<BG_ROWS;r++) {
        int nn=0, nc=obs->rows[r].count;
        if (nc<0 || nc>BG_CANDIDATES) return 0;
        for (i=0;i<n;i++) {
            int last=last_row(&beam[i],r), prev=last_row(&beam[i],last);
            BGPath gap=beam[i];
            gap.cost+=0.075f;
            insert(next,&nn,gap);
            for (j=0;j<nc;j++) {
                BGPath p=beam[i];
                float y=obs->rows[r].points[j].y, slope=0, old=0;
                if (!isfinite(y) || !isfinite(obs->rows[r].points[j].quality)) continue;
                if (last>=0) {
                    slope=(y-p.y[last])/(bg_distance(r)-bg_distance(last));
                    if (fabsf(slope)>1.65f) continue;
                    if (prev>=0) old=(p.y[last]-p.y[prev])/(bg_distance(last)-bg_distance(prev));
                    p.cost+=0.028f*slope*slope;
                    if (prev>=0) p.cost+=0.05f*(slope-old)*(slope-old);
                } else {
                    /* New paths must connect to a nearby visible guide line. */
                    if (r>3 || fabsf(y)>0.34f) continue;
                    p.cost+=0.8f*y*y;
                }
                p.cost+=0.012f*(1.0f-clamp(obs->rows[r].points[j].quality,0,1));
                p.y[r]=y; p.mask|=1u<<r; p.support++;
                insert(next,&nn,p);
            }
        }
        memcpy(beam,next,(size_t)nn*sizeof(BGPath)); n=nn;
    }
    for (i=0;i<n && used<BG_PATHS;i++) {
        int duplicate=0;
        if (beam[i].support<6 || last_row(&beam[i],BG_ROWS)<8) continue;
        for (j=0;j<used;j++) if (separation(&beam[i],&paths[j])<0.045f) duplicate=1;
        if (!duplicate) paths[used++]=beam[i];
    }
    return used;
}

/* Transform the old observed polyline into the current body frame. No world
   path, branch label or simulator pose is available to this function. */
static BGPath advance(const BGPath *p, float ds, float da) {
    float s[BG_ROWS],y[BG_ROWS],cs=cosf(da),sn=sinf(da);
    BGPath out; int i,j,n=0;
    memset(&out,0,sizeof(out));
    for (i=0;i<BG_ROWS;i++) if (p->mask & (1u<<i)) {
        float a=bg_distance(i)-ds,b=p->y[i];
        s[n]=cs*a+sn*b; y[n]=-sn*a+cs*b;
        if (n>0 && s[n]<=s[n-1]) continue;
        n++;
    }
    for (i=0;i<BG_ROWS;i++) {
        float q=bg_distance(i);
        for (j=1;j<n;j++) if (s[j-1]<=q && q<=s[j]) {
            out.y[i]=y[j-1]+(y[j]-y[j-1])*(q-s[j-1])/(s[j]-s[j-1]);
            out.mask|=1u<<i; out.support++; break;
        }
    }
    return out;
}
static float overlap_error(const BGPath *a,const BGPath *b,int prefix_only) {
    float sum=0; int n=0,i;
    for (i=2;i<(prefix_only?8:10);i++) if ((a->mask & b->mask) & (1u<<i)) {
        sum+=fabsf(a->y[i]-b->y[i]); n++;
    }
    return n>=3 ? sum/n : 0.0f;
}
static float bend(const BGPath *p) {
    float near=(bg_path_at(p,0.48f)-bg_path_at(p,0.22f))/0.26f;
    float far=(bg_path_at(p,1.15f)-bg_path_at(p,0.65f))/0.5f;
    return far-near;
}
static BGPath greedy(const BGObservation *obs,int route,int use_route) {
    BGPath p; int r,j;
    memset(&p,0,sizeof(p));
    for (r=0;r<BG_ROWS;r++) {
        float best=1e9f; int pick=-1,last=last_row(&p,r);
        if (obs->rows[r].count<0 || obs->rows[r].count>BG_CANDIDATES) break;
        for (j=0;j<obs->rows[r].count;j++) {
            float y=obs->rows[r].points[j].y;
            float d=fabsf(y-(last>=0?p.y[last]:0.0f));
            if (!isfinite(y)) continue;
            if (use_route && r>=7) d-=0.35f*route*y;
            if (d<best) { best=d; pick=j; }
        }
        if (pick>=0) {
            p.y[r]=obs->rows[r].points[pick].y; p.mask|=1u<<r; p.support++;
        }
    }
    return p;
}
BGOutput bg_step(BGState *st,const BGObservation *obs,int route,
                 float ds,float da,float dt,int method) {
    BGOutput out; BGPath prior,pending,choice;
    float best=1e9f,second=1e9f; int i,pick=-1,temporal,prefix_only;
    memset(&out,0,sizeof(out)); memset(&prior,0,sizeof(prior));
    if (!st || !obs || route<-1 || route>1 || method<0 || method>BG_NO_PREFIX ||
        !isfinite(ds) || !isfinite(da) || !isfinite(dt) || dt<=0 || dt>0.25f ||
        ds<0 || ds>0.6f || fabsf(da)>0.6f) return out;
    for (i=0;i<BG_ROWS;i++) {
        int j;
        if (obs->rows[i].count<0 || obs->rows[i].count>BG_CANDIDATES) return out;
        for (j=0;j<obs->rows[i].count;j++)
            if (!isfinite(obs->rows[i].points[j].y) ||
                !isfinite(obs->rows[i].points[j].quality)) return out;
    }
    temporal=method>=BG_TEMPORAL;
    prefix_only=method!=BG_NO_PREFIX;
    if (st->has_previous) prior=method==BG_NO_MOTION ? st->previous : advance(&st->previous,ds,da);
    if (method<=BG_GREEDY_ROUTE) {
        choice=greedy(obs,route,method==BG_GREEDY_ROUTE);
        if (choice.support>=6) { out.paths[0]=choice; out.count=1; pick=0; }
    } else {
        out.count=bg_search(obs,out.paths);
        for (i=0;i<out.count;i++) {
            float b=bend(&out.paths[i]);
            float score=out.paths[i].cost;
            score+=route==0 ? 0.16f*fabsf(b) : -0.22f*clamp(route*b,-1,1);
            if (temporal && st->has_previous)
                score+=0.10f*clamp(overlap_error(&prior,&out.paths[i],prefix_only)/0.12f,0,2);
            if (score<best) { second=best; best=score; pick=i; }
            else if (score<second) second=score;
        }
    }
    if (pick<0) {
        st->missing_time+=dt; st->has_pending=0; st->pending_time=0;
        if (temporal && st->has_previous && st->missing_time<=0.20f && prior.support>=3) {
            out.selected=prior; out.status=BG_HOLD; out.speed=0.30f;
            st->previous=prior;
        } else { st->has_previous=0; return out; }
    } else {
        st->missing_time=0; choice=out.paths[pick];
        out.margin=out.count>1 ? second-best : 1.0f;
        out.discrepancy=st->has_previous ? overlap_error(&prior,&choice,prefix_only) : 0;
        out.status=BG_TRACK;
        /* Confirmation applies to track replacements, not to every frame.
           Pending motion is still propagated while evidence accumulates. */
        if ((method==BG_FULL || method==BG_NO_MOTION || method==BG_NO_PREFIX) && st->has_previous &&
            out.discrepancy>0.055f && prior.support>=5) {
            pending=method==BG_NO_MOTION ? st->pending : advance(&st->pending,ds,da);
            if (st->has_pending && overlap_error(&pending,&choice,prefix_only)<0.035f)
                st->pending_time+=dt;
            else st->pending_time=dt;
            st->pending=choice; st->has_pending=1;
            if (st->pending_time<0.12f) {
                choice=prior; out.status=BG_PENDING;
            } else { st->has_pending=0; st->pending_time=0; }
        } else { st->has_pending=0; st->pending_time=0; }
        out.selected=choice; st->previous=choice; st->has_previous=1;
        out.speed=out.status==BG_PENDING ? 0.55f : 0.95f;
    }
    {
        float target_s=0.48f,target_y=bg_path_at(&out.selected,target_s);
        float curvature=2.0f*target_y/(target_s*target_s+target_y*target_y);
        out.steering=clamp(atanf(0.26f*curvature),-0.48f,0.48f);
        out.speed=fminf(out.speed,0.95f/(1.0f+0.45f*fabsf(curvature)));
    }
    return out;
}
