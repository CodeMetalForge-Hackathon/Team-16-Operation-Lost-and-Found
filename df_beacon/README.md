# Team 16 DF beacon (ESP32-S3)

A homing beacon for search and rescue. The ESP32-S3 transmits a steady, recognisable 2.4 GHz signal on
**one fixed channel (Wi-Fi channel 11)** so rescuers can direction-find to it.

- **Wi-Fi:** raw 802.11 beacon frames, 50 per second, SSID `TEAM16-DF`, channel 11 (2462 MHz).
- **BLE:** advertising every 20 ms under the name `TEAM16-DF`.
- Settings live in `platformio.ini` (`BEACON_CHANNEL`, `BEACON_RATE_HZ`, `BEACON_SSID`, `ENABLE_BLE`).
- It never hops channels and never joins a network. Do not add an amplifier or high-gain antenna to the
  transmitter; stock power is within the unlicensed limits.

## Build and flash

```
pip install platformio            # use Python 3.12 or older
pio run                           # build
pio run -t upload --upload-port COM6
pio device monitor -p COM6        # shows MAC, channel and "beacons sent N"
```

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
