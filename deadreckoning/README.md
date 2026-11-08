# INS coast (drone dead reckoning)

The pod's inertial navigation module. It **coasts between lidar position fixes**: an MPU9250 gives acceleration,
the drone's flight controller gives orientation, and the lidar terrain match (another module) gives a `pose_fix`
about once a second. Output is the `ins_coast` message from `SKILL.md` (`t, x, y, z, covariance`) plus decimal-degree
`lat`/`lon`.

## What to expect (simulated 100 m flights, `python evaluate.py`)

| Case | Error |
|---|---|
| Perfect sensors + perfect orientation | 0.0 m (checks frames and integration) |
| Realistic MPU9250, flight-controller orientation off by 0.5 to 2 deg, **coast only** | about 160 to 235 m after 60 s |
| Same + lidar fix every 1 s (+/-5 m) | about 5 m RMS, 10 m worst |
| Same, lidar drops out for 60 s | reaches about 120 m (team budget: 250 m per minute) |

Limit: these coast-error numbers are from simulation, not a flight. They check the logic and the order of magnitude, not
the real hardware. Real numbers need the IMU on the drone. Accelerometer bias and orientation error dominate:
1 degree of orientation error fakes 0.17 m/s^2 of sideways acceleration.

## Walking demo mode (hand-carried sensor, no drone)

Set the dashboard's mode to **Walking** (or `GET /mode?mode=walking`). It counts steps (peaks of the vertical
acceleration), moves 0.7 m per step along the heading, and uses the board's own gravity for tilt. Hold the board
level with its x arrow pointing the way you walk, and keep it still for the first 2 seconds (gyro and gravity
calibration). Simulated 56 m square: all 80 steps counted, 0.5 m (0.8%) from the true finish.

**Indoors, turn the compass off** (dashboard checkbox, or `GET /magnetometer?on=0`) and set the **start heading**:
a magnetometer that is 12 microtesla off (normal near metal) made the same walk 33% wrong, while gyro-only heading
from a set start heading stayed at 0.8%. A lidar scan match can also correct heading: POST `/fix` with
`"heading": degrees` along the x/y.

## Run it

```
python dr_run.py --sim                      # simulated flight, no hardware
python dr_run.py --serial COM5              # real MPU9250 stream (pip install pyserial)
python dr_run.py --replay recording.csv     # recorded stream
python evaluate.py                          # error numbers above (drone)
python test_walk.py                         # walking-mode check (simulated square walk)
```

Serves `:8084` (see the top of `dr_run.py` for every endpoint). In the dashboard the **Drone / INS** panel sets
the start coordinate (decimal degrees, or degrees-minutes-seconds text) and shows the position. Until the IMU is
wired to the Pi, run the service on a PC and type that PC's address into the panel's "INS service address" box.

## Inputs (line format over serial or a replay file; g, deg/s, microtesla; t in ms)

```
I,t_ms,ax,ay,az,gx,gy,gz,mx,my,mz      MPU9250 sample, about 100 Hz  (body: x forward, y left, z up)
Q,t_ms,w,x,y,z                         orientation from the flight controller (MAVLink / PX4 / ArduPilot, NED/FRD)
L,t_ms,agl_m                           lidar height above ground (tells the INS the drone has really landed)
F,t_ms,x_m,y_m,z_m,sigma_m             pose_fix from the lidar terrain match, in the map frame
```
Fixes can also be POSTed to `/fix` as `{"x","y","z","sigma"}` or `{"lat","lon","alt","sigma"}`.

## Notes and limits

- **Frames:** `x, y` are metres east/north in the map frame; `start_xy` is where the drone starts in it (default 0,0,
  matching the terrain-localizer `synthetic_hill` map). Flight-controller quaternions are converted from NED/FRD.
- **Without orientation input** it falls back to its own gyro + accelerometer + magnetometer filter, which is much
  worse in flight (the accelerometer reads thrust, not gravity). Feed it the drone's orientation.
- The filter treats the axes as independent and treats accelerometer error as noise, so its 1-sigma is a rough
  guide. A fix that disagrees with the estimate by more than 6 sigma is rejected.
- Changing the start while the drone is airborne makes the filter uncertain so fixes pull it in (about 10 s).
- The MPU9250 magnetometer is only used on the ground (motors corrupt it in flight).
- Drones fly only with Code Metal staff, in daytime.
