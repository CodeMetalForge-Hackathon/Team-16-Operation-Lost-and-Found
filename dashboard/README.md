# Team 16 live dashboard

Start the local dashboard from the repository root:

    python dashboard/serve.py

Open the local address printed by the server, enter the Raspberry Pi hostname
or IP address, and select **Connect screens**. The browser remembers the address
locally.

The dashboard displays the Pi-hosted visible-camera feed on port 8080 and
thermal feed on port 8081. Each feed must already be running on the Pi.

When it is time to host the dashboard on the Pi, copy this directory there and
run:

    python3 serve.py --host 0.0.0.0 --port 8090

When opened from the Pi-hosted address, the dashboard automatically uses that
same host for both sensor feeds.
