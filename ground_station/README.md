# Ground station map

This is one browser page for the whole team. It shows the search probability map, the pod track, detections, beacon drops and RF hits on the height map, next to the Pi's live camera feed. It runs offline, using only Python, numpy, matplotlib and scipy.

## Run it (from the repo root)
```
python ground_station/server.py                                   # add --camera http://<pi-address>:8080/ for the Pi feed
```
Then open http://localhost:8000 in a browser. The page opens on the White Mountains with no search running. Start one from the **Run a simulation** panel. **Reset search** goes back to this empty start screen. To open with a search already loaded, add `--run out/<folder>`. A teammate on the same Wi-Fi can use http://<laptop-ip>:8000.

The background is the team's QGIS overview map, `data/wmnf_demo_map.png` + `.pgw` (EPSG:26919). Swap it with `--basemap other.png`. The page buttons switch between the **White Mountains** overview, the **Search area**, and **Follow pod**. Scroll to zoom and drag to pan. All drawing is in UTM 19N, the same CRS as QGIS.

## Run a simulation anywhere on the map
Use the **Run a simulation** panel on the page. Either pick a trail, or choose "Click a point on the map…" and click where the person was last seen. Then press **Start**. The server builds a new search map there with `search_hotspots/hotspots.py`, which reads `data/height_map.tif` and takes about 10–30 s. It then flies `simulate_flight.py` over it. Each place is saved in `out/sim_<name>/`. "Hiker hiding spot" takes a number to repeat a run exactly; leave it blank for a random spot.

### Where the simulated hiker hides
- **Realistic** (default): the hiding spot is chosen independently of the search model. It takes a real case from `hiker_distances.json`, which records how far each of 36 real White Mountains finds was from the route that person was on (median ~270 m, up to 6.6 km). That distance is applied in a random direction from a random point on the route.
- **Follows our model**: the hiding spot is drawn from the search model's own probability map. This is the best case.

### Beacons
The pod carries 10 beacons (`--beacons N` on the server changes this). It drops one on every sighting:
- **Confirmed** (red star, "Bn · HIKER"): the camera saw a person. The search ends and the pod returns.
- **Possible sighting** (dark star with an orange outline, "Bn · check"): the camera saw an unnatural color, such as an orange patch. The pod marks it for a ground team and keeps searching. The simulator plants decoys like this in about 30% of boxes (`--decoy-rate`).

The "beacons left" stat counts down. If the pod runs out, later sightings are logged but not marked.

After a search ends, a pink ✕ shows where the hiker really was. Over 30 runs on test terrain, the pod found 53% of realistic hikers and 77% of model hikers within its 12 boxes.

## What it reads
It reads every `*.jsonl` file in `ground_station/logs/`, plus `detections.jsonl` in the repo root (which `picam/picam_color_anomaly.py` writes when run on the PC). The files are re-read every 2 s, so new lines show up live. Add more files with `--logs path1 path2 ...`.

| type | fields | notes |
|---|---|---|
| `pose_fix` / `ins_coast` | `t`, `lat`,`lon` **or** `x`,`y` (UTM 19N metres), `z`, `source` | `source:"lidar"` fixes are drawn as squares |
| `detection` | `t`, `cls`, `confidence`, optional `pose` | Without a pose, it is pinned to the track by nearest time (≤10 s) |
| `drop_event` | `t`, `pose`, `beacon_id`, optional `kind` (`confirmed`/`candidate`), `reason`, `beacons_left` | no `kind` = confirmed |
| `detection_note` | `t`, `note`, optional `pose` | e.g. a sighting the pod couldn't mark because it was out of beacons |
| `rf_hit` | `t`, `hz`, `power_db`, optional `pose` | |

The `search next` box comes from a Bayesian update. Cells the pod camera has covered (60 m footprint, POD 0.7) lose probability, and the box marks the most likely 500 m square that is left. The "probability cleared" stat is the share of the original map the pod has searched.

`simulate_flight.py` uses a placeholder beacon frequency (915 MHz). The real beacon must use the channel Stefan clears.
