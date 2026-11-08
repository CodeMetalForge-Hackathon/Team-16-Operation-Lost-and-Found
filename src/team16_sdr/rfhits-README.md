# rfhits -- phone-detect hit logs + live ports

Every phone-detect hit lands here, split by category. `rf_hub.py`
(systemd: `rf-hub.service`) tails `~/team16_sdr/monitor_hits.jsonl` and
`~/team16_sdr/rf_hits.jsonl` and fans records out by kind.

## Files (newline-delimited JSON, one hit per line)

| File | Source records |
|---|---|
| `bt_classic.jsonl` | `kind: "bt"` (public-block MAC sightings, hci0/hci1) |
| `ble.jsonl` | `kind: "ble"` (locally-administered MAC sightings) |
| `cellular.jsonl` | rf hits from the Pluto (`hz`, `power_db`, `pose`) |
| `wifi.jsonl` | `kind: "wifi-probe"` / `"wifi-assoc"` (Alfa monitor) |

## Live ports (127.0.0.1 only, localhost; no external access)

| Port | Category |
|---|---|
| 8091 | bt_classic |
| 8092 | ble |
| 8093 | cellular |
| 8094 | wifi |

Each port streams new hits as JSON lines the moment the hub ingests
them. Example client:

    nc 127.0.0.1 8092

Streams are **live only**: a client sees hits arriving after it
connects, never history. Multiple clients per port are fine.

## Record shape

Every line is the source record plus two hub fields:

- `t` -- capture time (float seconds, from the feed)
- `t_ns` -- hub-ingest time (int nanoseconds; sub-second tail lag)
- `cat` -- `"bt"`, `"ble"`, `"wifi"`, or `"cell"`

Identifiers and signal ride along from the source: `mac`, `name`,
`adapter`, `ssid` for bt/ble/wifi (`rssi` is the signal); `hz`,
`power_db`, `pose` for cell (`power_db` is the signal).

## Gotchas

- `rssi: 0` from hci0 is a controller artifact meaning "unknown", not
  a real reading.
- `run_pod.sh` truncates the source logs at each demo start; the hub
  notices the shrink and resets its read offsets automatically.

## Ideas (not implemented)

- Null-out `rssi: 0` from hci0 (measured garbage) instead of trusting it.
- A fifth combined port (8095) streaming all categories with `cat` as
  the discriminator.
- `--replay N` to send the last N file lines on connect once the
  dashboard wants history.
- Log rotation for `~/rfhits/*.jsonl` (grows unbounded; ~120 lines/min
  at demo rates).
