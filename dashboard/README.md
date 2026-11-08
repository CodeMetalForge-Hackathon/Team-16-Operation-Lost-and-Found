# Team 16 live dashboard

Start the local dashboard from the repository root:

    python dashboard/serve.py

Open the local address printed by the server, enter the Raspberry Pi hostname
or IP address, and select **Connect screens**. The browser remembers the address
locally.

The page shows five live panels. Each one uses the host you entered:

- Fused detection on port 8081 (visible camera cropped to the thermal field of view). Start it with `thermal/start_fusion.sh`.
- Thermal view on port 8082, from the same fusion process.
- Beacon health on port 8083: link, RSSI, SNR, packet loss, and battery. See `df_beacon/README.md`.
- Radio contacts on port 8085: closest contact, Wi-Fi, Bluetooth, BLE, and cellular energy. The feed reads the hub in `src/team16_sdr/rfhits-README.md`. The radio is receive-only.
- Inertial position on port 8084: latitude, longitude, and the coast from `deadreckoning/`. Until the IMU is on the Pi, run `dr_run.py` on the machine that has the board and type that address into the panel.

When it is time to host the dashboard on the Pi, copy this directory there and
run:

    python3 serve.py --host 0.0.0.0 --port 8090

When opened from the Pi-hosted address, the dashboard uses that same host for every panel.
