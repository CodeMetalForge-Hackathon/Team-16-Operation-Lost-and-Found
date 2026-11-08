# search_hotspots: where to look first

Builds a probability map for a missing hiker on one of the 35 trails and outputs ranked hot spots as JSON lines, GeoJSON and a heatmap.

```
pip install numpy scipy matplotlib openpyxl rasterio   # rasterio only needed for real DEM tiles
python hotspots.py --excel white_mountains_trails.xlsx --trail "Franconia Ridge Loop" --dem franconia_3dep.tif --out out/
python hotspots.py --excel white_mountains_trails.xlsx --trail "Mt. Chocorua" --ipp 43.9600,-71.2800   # last known point
```

Without `--dem`, it generates synthetic terrain so the pipeline still runs. The hot-spot positions from synthetic terrain are not real.

## Real terrain
Download a USGS 3DEP 1/3 arc-second (~10 m) GeoTIFF for the trail's box from The National Map downloader (apps.nationalmap.gov/downloader). Clip it to about 15 x 15 km around the trail. This can be the same tile the pod uses for lidar terrain matching.

## Model
```
P(cell) ∝ distance_prior × mobility × (1 + Σ w_i · layer_i)
```
| layer | what it captures | source |
|---|---|---|
| distance_prior | Mixture: 30% in a band hugging the trail, the rest a lognormal of distance from the route | `DIST_PCT_MI`: placeholder numbers, replace with Koester ISRID hiker tables |
| downstream | Follows D8 flow from route points downhill: the "walk downstream to civilization" behavior | DEM |
| stream | Proximity to drainages, taken from DEM flow accumulation (≥0.25 km² contributing area) | DEM |
| trail | Proximity to the planned route | gazetteer waypoints |
| alpine | Above treeline (~1,340 m) and near the route: whiteout and weather strandings | DEM |
| trap | Steep stream reaches (falls) and big-stream crossings | DEM |
| history | Kernel density of past incidents (lost/missing = 1.0, others = 0.25) | Excel `wmnf-incident-database`, geocoded by `gazetteer.csv` |
| mobility | Down-weights cells steeper than 35° | DEM |

Default weights (`WEIGHTS`) come from the 80 lost/missing records in the spreadsheet. About 30% mention water, about 34% were found on or near a trail, and about 44% mention a summit, ridge or treeline.

Hot spots come in two lists. `zone: "trail"` gives the best trail segments, which a hasty team covers first. `zone: "off-trail"` gives peaks more than 250 m from the route. Each hot spot carries `poa_250m` (probability inside a 250 m window) and `reasons` (its top contributing layers).

## Pod loop
After the drone sweeps an area and finds nothing, call `bayes_update(prob, searched_mask, pod)` and re-rank. Probability then shifts to the cells nobody has searched yet.

## Caveats
- `gazetteer.csv` coordinates are approximate. Check them against USGS or OSM before relying on them.
- The distance percentiles are placeholders. Replace them with ISRID numbers.
- The model has not been validated against real find locations. A good next step is to back-test it on cases in the missing_persons tab that have known find locations.
