# Direction finder

Ground tool for a strong steady LoRa radio at 915 MHz. One NESDR and a directional antenna. This tool only receives.

The app tunes to 915 MHz, checks that the signal is a LoRa chirp, and turns antenna headings into a bearing.

1. From `src/team16_sdr`, install numpy with `py -3 -m pip install -r requirements.txt`.
2. Plug in the NESDR. If Windows has no driver for it, install WinUSB for USB ID 0BDA:2838 with [Zadig](https://zadig.akeo.ie/).
3. Run `py -3 -m df` while the radio is transmitting.
4. Aim the antenna, pause a second, and mark the heading it is pointing. Two headings say which way is stronger. Three or more produce a peak bearing.

`py -3 -m df --demo` uses a synthetic 915 MHz chirp whose true bearing is 60 degrees.

`py -3 -m df --check` tunes to 915 MHz, reads one block, and prints the power. No window.

Each mark is appended to `df-log.jsonl` in the current directory. Power is dBFS at a fixed tuner gain, so headings can be compared. It is not a calibrated field strength.
