# Quick-Complex Authoritative Map

Generated from the active quick-complex scene YAML and transformed mesh/model geometry.
The retained PNGs are review figures with axes/borders, not raw coordinate rasters.
Geometry comes from the retained CSV/YAML/mesh chain; visual styling is
cosmetic and clipped to the same footprints.

## Outputs

- `01_clean_visual_base.png`: clean visual base; no planner obstacles, no route points.
- `02_visual_planner_obstacles.png`: same visual base plus planner-visible obstacle footprints; no route points.
- `03_visual_planner_route_p0_p8.png`: same visual/planner layers plus the current P0-P8 route.
- `visual_mesh_footprints.csv`: transformed mesh centers and footprint bounds used to render visual objects.
- `route_waypoints.csv`: current P0-P8 route point positions, yaw, radius, hold, and timeout data.
- `planner_obstacles.csv`: planner-visible static/dynamic obstacle positions and dimensions.
- `map_layer_index.csv`: PNG-to-CSV layer/source index. No PNG should be treated as standalone evidence.
- `map_manifest.json`: source paths and coordinate extent.

Legacy descriptive PNG names were removed from the retained display path to
avoid old-noise lookup mistakes. Use only the numbered outputs above for review.

## Current Drawing Standard

- Use the real YAML/mesh/planner geometry for position, scale, yaw, and footprint.
- Keep readable top-down ship details and island contour shading inside those existing footprints.
- Keep visual-world styling separate from planner-collision geometry and metrics.
- Regenerate figures from this script instead of editing PNGs by hand.

## Layer Meaning

- Visual layer: transformed OBJ/SDF mesh footprint, used for visual semantics such as ship deck / shoreline placement.
- Readable icon overlay: drawn from the same YAML centers/yaws and mesh-derived bridge centerlines; it is cosmetic only and does not replace the geometry source.
- Planner layer: YAML buoys, boxes, and dynamic obstacle footprints that are visible to the planner/evaluator.

No scene YAML, route waypoint, obstacle, or planner parameter was changed by this render step.

## Plotting Rule

For final planned-vs-flown plots, draw the map layers, planned P0-P8 route,
and selected trajectory on the same axes from retained CSV/JSON inputs.
Use numbered current outputs for display. This F250 tutorial package keeps its
retained display plot at
`../../../evidence/expected_route/f250_historical_planned_vs_flown.png`.
Do not overlay trajectories onto a previously rendered figure PNG.

## RViz Display Layer

RViz scene markers may publish additional static mesh context from the same
scene YAML `visual_vessels` entries, such as ships, islands, bridges, and wind
turbines. Those markers are visual context only and do not change planner cloud
geometry, route geometry, static obstacle safety gates, or this map authority.
