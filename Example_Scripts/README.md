# Example Python Scripts

The offboard Python code in this directory can be used for both simulation and on hardware.

The instructions of running this code in simulation mode using PX4 and ArduPilot is as follows:

Terminal 1 (T1)
```
cd ~/   
./QGroundControl.AppImage
```

T2
```
cd ~/PX4-Autopilot   \
make px4_sitl gz_x500
```
or, Ardupilot
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

*Then put QGC in "Guided" mode if using ArduCopter

T3
```
cd ~/Mavlink-Main
python3 offb_pymav.py
```

## Functional Descriptions

### offb_pymav.py

A drone is set at an initial lat, long, and altitude by the line:

mission_waypoints.append(mission_item(0, 0, 45.66874762129226, -111.07818877582056, target_altitude))

This is actually the only waypoint set. The drone will rise by target_altitude (5 meters in code presently). Once the drone has reached this waypoint, it then returns to home and mission is complete!

Sequence of events are:

- Make a mavlink communication connection to QGroundControl and ArduPilot (simulated drone).
- Wait for a system heartbeat.
- Create and upload mission waypoints (done in a single line for this example).
- arm()
- takeoff()
- Wait 10 seconds after reaching single waypoint (altitude) then
- land()



 
