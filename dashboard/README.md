# Team 16 live dashboard

Start the local dashboard from the repository root:

    python dashboard/serve.py

Open the local address printed by the server, enter the Raspberry Pi hostname
or IP address, and select **Connect screens**. The browser remembers the address
locally.

The dashboard displays the Pi-hosted fused detection view on port 8081 (visible camera cropped to the
thermal field of view) and the thermal view on port 8082. Start them on the Pi with
`thermal/start_fusion.sh`.

When it is time to host the dashboard on the Pi, copy this directory there and
run:

    python3 serve.py --host 0.0.0.0 --port 8090

When opened from the Pi-hosted address, the dashboard automatically uses that
same host for both sensor feeds.
