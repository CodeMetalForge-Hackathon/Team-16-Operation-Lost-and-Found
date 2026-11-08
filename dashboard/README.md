# Team 16 live dashboard

Start the local dashboard from the repository root:

    python dashboard/serve.py

Open the local address printed by the server, enter the Raspberry Pi hostname
or IP address, and select **Connect screens**. The browser remembers the address
locally.

The page shows six live panels. Each one uses the host you entered:

- Fused detection on port 8081 (visible camera cropped to the thermal field of view). Start it with `thermal/start_fusion.sh`. **Recalibrate** aligns the two cameras.
- Thermal view on port 8082, from the same fusion process. The image is rotated 180 degrees at the source because the camera is mounted upside down.
- Pi health on port 8087: CPU (total and per core), temperature, memory, throttling, whether the Hailo is on the PCIe bus, detector frame rate, detector restarts in the last 15 minutes, and the busiest processes. The feed is `pi_stats.py`. It reads `/proc`, `/sys` and the detector's log and never opens the Hailo.
- Radio contacts on port 8085: closest contact, Wi-Fi, Bluetooth, BLE, and cellular energy. The feed (`sdr_feed.py`) reads the hub in `src/team16_sdr/rfhits-README.md`. The radio is receive-only.
- Lidar room map on port 8086: the saved room map, the lidar position and heading, and the track so far. The lidar laptop POSTs a JSON document to `map_feed.py` (outline cells, `origin`, `meters_per_pixel`, `pose`). The feed keeps every posted position as `trail` and saves it in `map_feed_state.json`, so a new post never erases the last. **Rotate map** turns the view in 90 degree steps, **Clear track** forgets the trail. Lidar only, no fusion with the inertial position.
- Inertial position on port 8084: latitude, longitude, and the coast from `deadreckoning/`. Until the IMU is on the Pi, run `dr_run.py` on the machine that has the board and type that address into the panel.

Start the feeds on the Pi (each has a `start_*.sh` and `stop_*.sh`, read-only apart from the map feed's state file):

    ./start_sdr_feed.sh
    ./start_pi_stats.sh
    ./start_map_feed.sh

The Pi starts the dashboard, the SDR feed, the INS, the stats feed, the map feed and the detector at boot from its crontab (`@reboot` with a short sleep each). The map feed runs in a small loop so it restarts itself if it exits.

When it is time to host the dashboard on the Pi, copy this directory there and
run:

    python3 serve.py --host 0.0.0.0 --port 8090

When opened from the Pi-hosted address, the dashboard uses that same host for every panel. Open the Pi's own address over plain http. A page served over https cannot call the Pi's http feeds.

The page is plain light by default and follows the system dark setting. All colors are tokens at the top of `styles.css`.
