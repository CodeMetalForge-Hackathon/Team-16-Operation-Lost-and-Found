# Ground station map

This is one browser page for the whole team. It shows the search probability map, the pod track, detections, beacon drops and RF hits on the height map, next to the Pi's live camera feed. It runs offline, using only Python, numpy, matplotlib and scipy.

## Run it (from the repo root)
```
python search_hotspots/hotspots.py --trail "Franconia Ridge Loop" --out out/franconia_real     # once; writes grid.json etc.
python ground_station/simulate_flight.py --run out/franconia_real --live 20                    # simulated pod flight (optional)
python ground_station/server.py --run out/franconia_real --camera http://<pi-address>:8080/
```
Then open http://localhost:8000 in a browser. A teammate on the same Wi-Fi can use http://<laptop-ip>:8000.

The background is the team's QGIS overview map, `data/wmnf_demo_map.png` + `.pgw` (EPSG:26919). Swap it with `--basemap other.png`. The page buttons switch between the **White Mountains** overview, the **Search area**, and **Follow pod**. Scroll to zoom and drag to pan. All drawing is in UTM 19N, the same CRS as QGIS.

## What it reads
It reads every `*.jsonl` file in `ground_station/logs/`, plus `detections.jsonl` in the repo root (which `picam/picam_color_anomaly.py` writes when run on the PC). The files are re-read every 2 s, so new lines show up live. Add more files with `--logs path1 path2 ...`.

| type | fields | notes |
|---|---|---|
| `pose_fix` / `ins_coast` | `t`, `lat`,`lon` **or** `x`,`y` (UTM 19N metres), `z`, `source` | `source:"lidar"` fixes are drawn as squares |
| `detection` | `t`, `cls`, `confidence`, optional `pose` | Without a pose, it is pinned to the track by nearest time (≤10 s) |
| `drop_event` | `t`, `pose`, `beacon_id` | |
| `rf_hit` | `t`, `hz`, `power_db`, optional `pose` | |

The `search next` box comes from a Bayesian update. Cells the pod camera has covered (60 m footprint, POD 0.7) lose probability, and the box marks the most likely 500 m square that is left. The "probability cleared" stat is the share of the original map the pod has searched.

`simulate_flight.py` uses a placeholder beacon frequency (915 MHz). The real beacon must use the channel Stefan clears.
