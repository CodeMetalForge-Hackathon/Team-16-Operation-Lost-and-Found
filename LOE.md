# Lines of effort

Team 16, Operation Lost and Found. A pod for a missing person in the White Mountains when GPS, cell, and fixed comms are gone. Four lines of effort. Each one below says what it does, where the code lives, and where it stops.

## LOE 1. Locate the pod without GPS

The pod coasts on an IMU and resets that coast with a lidar match against a heightmap. No GPS in the loop.

The coast is `deadreckoning/`. An MPU9250 supplies acceleration. Orientation should come from a flight controller in the air. In simulation, realistic sensor error and no fix leave the position 160–235 m off after 60 seconds. A lidar fix every second holds about 5 m RMS, with a worst case near 10 m. A 60-second dropout walks out to about 120 m. The budget is 250 m per minute of coast. Walking mode, also simulated, counted all 80 steps of a 56 m square and finished 0.5 m off. Turn the compass off indoors. Twelve microtesla of error, ordinary near metal, made that same walk 33% wrong.

The mountain match is `src/terrain_localize/`. It correlates a 3D lidar cloud with a prior heightmap. No initial pose. A check on three known poses must recover each within 30 m and 15 degrees. The map in the repo is a synthetic 2 km hill. The Gazebo heightmap shader does not compile on this laptop's GPU, so the repeatable demo is the ray-cast. This has not flown, and it has not been run on a White Mountain pass.

The rig to carry around a table is `src/lidar_tilt/` and `firmware/lidar_tilt/`. An RPLidar C1 stays a 2D lidar. A hobby servo on a XIAO steps from -30 to 30 degrees and stacks scans into a room-scale occupancy grid. The fix is x, y, z, and yaw. Ranges closer than 0.15 m or past 12 m are dropped.

Alexander MacConnell owns the lidar and the terrain match. Roman owns the coast.

## LOE 2. Detect a person

Three detectors, then one fused label.

The color camera (`picam/`) learns the scene, then circles a color it has not been shown. Brightness alone does not trip it.

The thermal camera is an HT-203U (`thermal/`). Seventy-four labeled frames, 27 of them a person. A random forest on six hand-built features scores 0.73 AUC on held-out sessions and about 66% accuracy. YOLOv8s on the Hailo-8 scores 0.80 AUC on the same frames, which it never trained on, in about 15 ms. At the usual 0.5 cutoff that is 72% accuracy. At 0.05 it is 85%. The network was trained on color photos, so scores on thermal run low and false alarms are expected. The Hailo is not faster than the tiny forest. It locates the person and leaves the CPU free.

Fusion (`thermal/fusion_pi.py`) runs both cameras and one Hailo in a single process. Labels are HUMAN (shape plus heat), THERMAL ONLY (heat, no visible person), NO HEAT (a body with no heat; treat as a decoy until it is not), and UNCONFIRMED. Weights are hand-set. Accuracy of the fused label is unmeasured. Alignment is per mount. Without a calibration the overlay says so. A watchdog restarts the detector if it stops writing frames.

Seeing through canopy (`thermal/thermal_stitch.py`) is a simulation. At 20 m, 0.3 m of pose error still holds a warm target. 0.5 m loses it. The camera is assumed nadir over flat ground.

Roman owns this line.

## LOE 3. Decide where to look, and keep one picture

`search_hotspots/` builds a probability-of-area map for one of 35 trails.

Probability in a cell follows distance from the planned route, times whether the slope can be walked, times water, trail, alpine ground, terrain traps, and past incidents. Weights from the incident sheet: downstream 1.5, stream 0.6, trail 1.0, alpine 0.8, trap 0.8, history 1.0. The sheet holds 547 incidents, 80 of them lost or missing. About 30% of those mention water, about 34% were found on or near a trail, and about 44% mention a summit, a ridge, or treeline. Treeline here is 1,340 m.

Franconia Ridge Loop has a heatmap from a real elevation tile. Trails run without a tile use synthetic terrain, and those pins are not real places. Distance bands are placeholders. Gazetteer coordinates are approximate. The model has not been scored against known find locations. `bayes_update` moves probability onto cells a sweep has cleared. That update has not been run on a live sweep.

The live picture for a judge is `dashboard/`: fused camera, thermal camera, beacon health, radio contacts, and the inertial position, on one screen.

Lauren Berning owns the search map. Roman owns the dashboard.

## LOE 4. Hand the find to a team on foot

Two pieces. A beacon they can walk to, and a log of other transmitters at the pod's pose.

The beacon is `df_beacon/`, a LilyGO LoRa32 on its own battery. LoRa at 915 MHz, spreading factor 10, 125 kHz, 17 dBm, one packet a second. The packet is an id, a sequence, and uptime. A gap in the sequence is a missed packet. The same board also sends a Wi-Fi beacon and a Bluetooth name, `TEAM16-DF`, so a phone can walk toward it in a room. A phone cannot hear the LoRa. LoRa is the field signal. The ground set is a second board, or a Pluto watching 915 MHz. This build is carried and set down. A release servo and a parachute are not in the repo. Put the antenna on before power. Clear the channel with the hardware mentor before the first transmission.

The pod SDR is `src/team16_sdr/`. It stays receive-only. A hit is energy above the local noise, with time, frequency, and power, and the pose when one exists. A second Pluto can watch the beacon while the first keeps scanning. A monitor radio logs Wi-Fi probes, associations, and Bluetooth advertisements. The screen ranks contacts strongest first. A phone with no tower it can decode stays quiet. What this catches is a phone that still has a tower, or just lost one. A handset is about a fifth of a watt, so expect it nearby. The Pi's own Wi-Fi and Bluetooth can send a short invitation so a nearby handset answers. Those are the Pi's radios, not the SDR.

Roman owns the beacon. Patrick McAuliffe owns the radios.

## Left out on purpose

The Pluto can transmit, so three further ideas were physically possible. None of them are in the repo.

- A fake cell, so phones would attach to the pod instead of a carrier.
- Collection of IMEI and IMSI from that attach.
- An alert pushed onto the handset.

Impersonating a cellular network, collecting those identifiers, and writing a message onto someone else's phone are illegal for this team. An unlicensed band does not make them legal. The only transmitter this project will key is the beacon.
