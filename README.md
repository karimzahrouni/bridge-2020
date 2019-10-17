# bridge-2020
### Dennis Evangelista, 2019
RPi bridge node to NMEA 2000, plus Spektrum receiver

## Push Test 1
For Push Test 1 (PT1) this node is not needed.

## Push Test 2
For Push Test 2 (PT2) this node will receive Spektrum RC signals from the DX-6 or DX-9 for rudder and mainsail (sheet) commands and a mode switch. Rudder is to be moved to the knob; sheet will be mapped to the throttle. This node will also serve as the bridge to the RPi (via ROS and rosserial) to allow the RPi to log commands as a "bell book" / "deck log". 

## Applicability
USNA Sailbot AY20 Hull 14 mod 3 and later.
