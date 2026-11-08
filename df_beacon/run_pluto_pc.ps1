# Run the PlutoSDR receive-only monitor on this PC and send readings to the Pi's dashboard feed.
# Needs: Analog Devices PlutoSDR drivers + libiio for Windows (see df_beacon/README.md), Python 3.12 venv at
#   C:\Users\88rom\team16_pluto_venv   (py -3.12 -m venv; pip install numpy "pylibiio==0.25")
# Usage:  .\run_pluto_pc.ps1            or      .\run_pluto_pc.ps1 -Pi 192.168.8.163 -Uri ip:192.168.2.1
param(
  [string]$Pi  = "192.168.8.163",
  [string]$Uri = "usb:",           # use "ip:192.168.2.1" if the Pluto shows up as a network adapter
  [double]$Gain = 40
)
$env:PATH = "C:\Users\88rom\team16_pluto_libiio\bin;" + $env:PATH   # libiio 0.26 DLLs (see README)
$py = "C:\Users\88rom\team16_pluto_venv\Scripts\python.exe"
& $py "$PSScriptRoot\pluto_rx.py" --uri $Uri --gain $Gain --sink "post:http://${Pi}:8083/ingest"
