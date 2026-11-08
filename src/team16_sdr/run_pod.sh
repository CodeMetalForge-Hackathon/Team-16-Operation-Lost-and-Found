#!/usr/bin/env bash
# Pod demo: bait AP + Alfa monitor + Pluto uplink dwell + BT coax, N s,
# then a ranked contact report. Receive-only except the Pi's own TX
# (bait beacon on wlan0, inquiry/scan on hci0); never touches the
# Pluto TX chain (altvoltage1 / ensm_mode).
set -u
# tcpdump (wifi feed) needs root; re-exec the whole script under sudo so
# every child and log file share one owner.
[ "$(id -u)" -ne 0 ] && exec sudo -E -n bash "$0" "$@"
cd "$(dirname "$0")"

IFACE=${IFACE:-wlx00c0cabea289}
SECONDS_WAIT=${SECONDS_WAIT:-60}
MON=monitor_hits.jsonl
RF=rf_hits.jsonl
CONTACTS=phone_contacts.jsonl
rm -f "$MON" "$RF" "$CONTACTS"   # demo window only
systemctl is-active --quiet rf-hub || sudo -n systemctl start rf-hub

pkill -9 -f "[p]hone_scan.py" 2>/dev/null   # libiio: one buffer per device

# Alfa WiFi half -> monitor mode on the PARENT iface (virtual mon
# interfaces get stuck on 5825 MHz with this mt7921u card).
nmcli device disconnect "$IFACE" 2>/dev/null
ip link set "$IFACE" down
iw dev "$IFACE" set type monitor
ip link set "$IFACE" up
iw dev "$IFACE" set channel 1 || echo "WARN: channel set failed" >&2

python3 monitor_scan.py --iface "$IFACE" --adapter hci0,hci1 --out "$MON" &
MON_PID=$!
python3 phone_scan.py --uri usb: --gain 40 --out "$RF" &
RF_PID=$!
python3 bt_coax.py --adapter hci0 --out "$MON" &
COAX_PID=$!

cleanup() {
  kill "$MON_PID" "$RF_PID" "$COAX_PID" 2>/dev/null
  wait "$MON_PID" "$RF_PID" "$COAX_PID" 2>/dev/null
  # SIGTERM skips monitor_scan's finally; reap its tcpdump child too.
  pkill -f "[t]cpdump -i $IFACE" 2>/dev/null
}
trap cleanup EXIT

sleep "$SECONDS_WAIT"
cleanup
trap - EXIT

python3 phone_phone.py --monitor "$MON" --rf "$RF" --out "$CONTACTS" --once
python3 phone_phone.py --monitor "$MON" --rf "$RF" --report
