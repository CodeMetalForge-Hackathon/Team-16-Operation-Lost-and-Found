# team16_sdr

Receive-only SDR work for the pod.

Listen on the band a mentor clears. Log time, frequency, and power, and attach the pod pose when one exists. This folder does not transmit.

## Tools (run on the Pi, `hacker@192.168.8.163`, repo under `~/team16_sdr`)

Cellular uplink scanner (Pluto, RX-only): `python3 phone_scan.py --uri usb:` dwells
2 s per channel over the default cellular uplink list (2620/1750/850 MHz), prints
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
answering.

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
