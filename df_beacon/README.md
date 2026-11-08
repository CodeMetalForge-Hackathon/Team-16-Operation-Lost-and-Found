# Team 16 DF beacon

A homing beacon for search and rescue. It transmits steady, recognisable signals so rescuers can
direction-find to it. Receive side is receive-only (SDR or a second LoRa board).

| Board | Environment | Signals |
|---|---|---|
| **LilyGO LoRa32 915 MHz** (ESP32-PICO-D4 + SX1276), the field unit | `esp32-pico` | **LoRa 915 MHz** (main, long range), Wi-Fi ch 11, BLE |
| Same board as a receiver | `esp32-pico-rx` | listens on LoRa, prints RSSI/SNR of each beacon |
| ESP32-S3 dev board (first prototype) | `esp32s3` | Wi-Fi ch 11, BLE |

- **LoRa:** one packet per second (`T16DF,<id>,<seq>,<uptime s>`), 915 MHz, SF10, BW125 kHz, 17 dBm.
  Sub-GHz passes through foliage far better than 2.4 GHz, so this is the signal to search for.
  Packet loss shows up as gaps in `<seq>`.
- **Wi-Fi:** raw 802.11 beacon frames, 50/s, SSID `TEAM16-DF`, channel 11 (2462 MHz). Phone-testable.
- **BLE:** advertising every 20 ms as `TEAM16-DF`. Phone-testable.
- Settings (frequency, SF, power, channel, which signals are on) are the `-D` flags in `platformio.ini`.
- **Attach the 915 MHz antenna before powering the board.** Keep power within your country's limits.

## Build and flash

```
pip install platformio            # use Python 3.12 or older
pio run -e esp32-pico -t upload --upload-port COM7
pio device monitor -p COM7        # shows "lora sent N" rising once per second
```

For a receiver, flash a second board with `-e esp32-pico-rx`. Pins are the LilyGO T3 V1.6.1 layout
(SCK 5, MISO 19, MOSI 27, CS 18, RST 23, DIO0 26); change them in `platformio.ini` for other variants.

A phone cannot receive LoRa (no LoRa radio). The Wi-Fi and BLE tests below only check the 2.4 GHz side.

## Test it with your phone (as the receiver)

The phone is just a signal-strength meter: the closer you are to the beacon, the higher (less negative)
the dBm number. Roughly: -30 to -50 dBm is within a few metres, -60 to -70 is across a room or two,
-80 or lower is far or blocked.

### Android (Wi-Fi, the main test)
1. Install **WiFiAnalyzer** (open source, by VREM) from Google Play. Grant the location permission it asks
   for (Android needs it for Wi-Fi scans).
2. Power the board from USB. Wait about 5 seconds.
3. Open the app. You should see **TEAM16-DF** on **channel 11** in the AP list and the channel graph.
4. Tap it to see live signal strength. Walk toward and away from the board and watch the number change.
5. Android limits Wi-Fi scans to about 4 per 2 minutes, which makes the number update slowly. Turn that
   off: Settings > Developer options > **Wi-Fi scan throttling** off (Android 9 and later). Without
   it, the app still works but feels laggy.

### Android or iPhone (BLE, faster updates)
1. Install **nRF Connect** (Nordic Semiconductor, free) or LightBlue.
2. Scan. Find **TEAM16-DF** and watch its RSSI. It updates several times a second, so it is the better
   one for walking toward the beacon.
3. iPhones cannot show Wi-Fi signal strength for other networks (iOS blocks it), so on iPhone the BLE
   test is the one to use.

### How to direction-find with just a phone
1. Stand still and watch the number for 10 seconds, to see how much it jumps.
2. Hold the phone against your chest and **turn slowly in a full circle**. Your body blocks the signal
   when you face away, so the strongest reading is roughly toward the beacon.
3. Walk in that direction, and repeat the turn every 10 to 20 m. Move on the side where it gets
   stronger.
4. Expect jumps of +/- 5 to 10 dBm from reflections off walls and people. Average over a few seconds.
   Do not trust a single reading.

### Checks if it does not show up
- Nothing in the list: check the serial monitor. `beacons sent` should be rising. If `failed` rises, tell
  me what the monitor prints.
- Phone shows other networks but not this one: some phones hide networks with no security or no
  clients. Try BLE, or a second scanner app.
- Weak at a few metres: the board's built-in antenna is directional. Rotate the board and keep it
  away from metal and your hand.
