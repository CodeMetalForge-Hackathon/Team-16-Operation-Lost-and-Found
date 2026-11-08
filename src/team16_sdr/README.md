# team16_sdr

Receive-only SDR work for the pod.

Listen on the band a mentor clears. Log time, frequency, and power, and attach the pod pose when one exists. The SDR never transmits; the Pi's own radios (bait AP, Bluetooth inquiry) do.

## Tools (run on the Pi, `hacker@192.168.8.163`, repo under `~/team16_sdr`)

Cellular uplink scanner (Pluto, RX-only): `python3 phone_scan.py --uri usb:` dwells
2 s per channel over the default cellular uplink list (2535/1730/707 MHz =
B7/B4/B12), prints
`rf_hit` JSON lines (`t`, `hz`, `power_db`, `pose`) on energy above the noise floor,
and appends them to `rf_hits.jsonl`. Walk-log proximity: `python3 walk_log.py
--role log` (Enter to sample, type a position) then `python3 walk_log.py --role
report` — position inference is RSSI trend from walking, not antenna direction
finding; strongest uplink marks the phone's direction. WiFi/BT monitor (Alfa USB
card): `sudo python3 monitor_scan.py` logs WiFi probe/association requests from a
monitor interface plus Bluetooth/BLE sightings to `monitor_hits.jsonl`; the Pi's
onboard WiFi broadcasts the open bait AP `Low Quality Bait` so nearby phones probe and
associate. Bluetooth scan requests: `python3 bt_scan.py` runs 15 s active-scan
windows (BR/EDR inquiry + BLE scan requests) that provoke nearby devices into
answering. `monitor_scan.py --adapter` takes a comma list (default `hci0,hci1`):
one passive-sighting feed per adapter, each record stamped `"adapter"`; a missing
controller prints `ERROR: bt unavailable (hciN: no controller)` and the other
feeds run. On the Pi, `hci1` (the Alfa's MT7961 BT half) cannot come up —
`hciconfig hci1 up` wedges in D state (kernel NULL deref in
`btmtk_usb_hci_wmt_sync` on 6.8.0-1065-raspi) — so `hci0` sniffs and coaxes.
`python3 bt_coax.py` is the TX side: `discoverable on` + scan bursts on `hci0`
that make phones answer, one `coax-window` summary line per window.
Correlator: `python3 phone_phone.py` merges `monitor_hits.jsonl` with
`rf_hits.jsonl` into `phone_contacts.jsonl` (WiFi/BT contacts per MAC inside
`--window`, bait-SSID probes or `--min-hits` sightings qualify; cellular rf hits
in an uplink range become `kind: "cell"` contacts); `--report` ranks contacts
strongest-signal-first and labels the top one `closest`. Demo runner:
`bash run_pod.sh` brings up the Alfa monitor iface, starts all three feeds plus
the coaxer, waits `SECONDS_WAIT` (default 60 s), then prints the report. Bait AP:
`pod-bait.service` (oneshot, enabled) keeps `Low Quality Bait` on `wlan0`
channel 1 across reboots. Each Pluto is lock-guarded per device:
`phone_scan.py` holds an exclusive flock on
`/tmp/pluto-sdr-<serial6>.lock` (or `/tmp/pluto-sdr.lock` for bare
`usb:`) because libiio allows one buffer per device, so two feeds on
two Plutos coexist; a second instance on the SAME Pluto prints
`ERROR: Pluto busy (holder has <lock>)` and exits 1. Pin a device with
`--pluto-serial <s>` (resolves `usb:<bus>.<devnum>.5` via sysfs;
devnums change across reboots).

Honest scope: a phone transmits its PRACH preamble only after decoding a downlink
cell (LTE: PSS/SSS sync, PBCH/MIB, SIB1, SIB2 `RACH-ConfigCommon`; NR: SSB + SIB1)
— it needs a decodable cell, not a working network, and it power-ramps preamble
retransmissions whenever no Random Access Response follows. A phone in a true dead
zone with zero decodable carriers is silent (cell search is receive-only; 3GPP
periodic network re-attempts are minutes apart), so detection means the phone is
camped or at coverage edge. The phone is a 0.2 W talker, so expect audibility at
close range only. The repeatable demo trigger is an Airplane-mode toggle forcing an
attach burst. The Pluto is limited to cellular uplink bands; WiFi/BLE/Bluetooth
belong to the Alfa card and the Pi's onboard radios. The onboard AP and Bluetooth
scan requests DO transmit (2.4 GHz, low power) — they are Pi radios, not the SDR.
