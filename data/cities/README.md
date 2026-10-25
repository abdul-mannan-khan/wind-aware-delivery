# Real city geometry

Building heights rasterised at 1 m from OpenStreetMap by `sim/osm_city.py`, each a 640 m × 640 m area:

| File | Area | Building polygons (with height or level tag) | Tallest |
|---|---|---|---|
| `london.npz` | City of London, around Bank and Leadenhall Street | 451 (175) | 278 m |
| `manhattan.npz` | Midtown Manhattan, around Bryant Park | 415 (335) | 366 m |

A building without a height tag gets 3.2 m per level plus 2 m, or 12 m if it has neither tag. The `.png` files
show the height maps with the depot and the four customers. Rebuild them with
`python3 osm_city.py fetch london ../data/cities/london.npz` (needs internet).

Map data © OpenStreetMap contributors, available under the Open Database Licence (ODbL),
https://www.openstreetmap.org/copyright. These derived rasters are distributed under the same licence.
