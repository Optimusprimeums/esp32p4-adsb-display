/* Empty offline map, used when main/map_data.c has not been generated.
 * Run tools/gen_map.py for your receiver location to get coastlines,
 * runways and airport codes on the radar (see docs/offline-map.md). */
#include "map_data.h"

const float map_coast_pts[][2] = {{0.0f, 0.0f}};
const map_line_t map_coast_lines[] = {{0, 0, 0, 0, 0, 0}};
const int map_coast_line_count = 0;
const char map_coast_credit[] = "";
const map_airport_t map_airports[] = {{0, "", "", "", 0.0f, 0.0f}};
const int map_airport_count = 0;
const map_runway_t map_runways[] = {{0, 0, 0, 0, 0, 0}};
const int map_runway_count = 0;
