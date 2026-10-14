#ifndef BRANCHGUARD_H
#define BRANCHGUARD_H
#include <stdint.h>

#define BG_WIDTH 188
#define BG_HEIGHT 120
#define BG_ROWS 12
#define BG_CANDIDATES 6
#define BG_PATHS 8
#define BG_BEAM 24

/* Lateral coordinates are metres, positive to the vehicle's left. */
typedef struct { float y, quality; } BGPoint;
typedef struct { int count; BGPoint points[BG_CANDIDATES]; } BGRow;
typedef struct { BGRow rows[BG_ROWS]; } BGObservation;
typedef struct {
    float y[BG_ROWS], cost;
    unsigned int mask;
    int support;
} BGPath;
typedef struct {
    BGPath previous, pending;
    float missing_time, pending_time;
    int has_previous, has_pending;
} BGState;
typedef struct {
    BGPath paths[BG_PATHS], selected;
    float steering, speed, margin, discrepancy;
    int count, status;
} BGOutput;

enum { BG_NEAREST, BG_GREEDY_ROUTE, BG_SPATIAL, BG_TEMPORAL,
       BG_FULL, BG_NO_MOTION, BG_NO_PREFIX };
enum { BG_STOP, BG_TRACK, BG_PENDING, BG_HOLD };

void bg_init(BGState *state);
void bg_extract(const uint8_t *image, int stride, BGObservation *out);
int bg_search(const BGObservation *obs, BGPath paths[BG_PATHS]);
BGOutput bg_step(BGState *state, const BGObservation *obs, int route,
                 float distance, float yaw_change, float dt, int method);
float bg_distance(int row);
int bg_pixel_row(int row);
float bg_path_at(const BGPath *path, float distance);
#endif
