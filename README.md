# bridge-2020
### Dennis Evangelista, 2019
RPi bridge node to NMEA 2000, plus Spektrum receiver

## Push Test 1
For Push Test 1 (PT1) this node is not needed.

## Push Test 2
For Push Test 2 (PT2) this node will receive Spektrum RC signals from the DX-6 or DX-9 for rudder and mainsail (sheet) commands and a mode switch. Rudder is to be moved to the knob; sheet will be mapped to the throttle. This node will also serve as the bridge to the RPi (via ROS and rosserial) to allow the RPi to log commands as a "bell book" / "deck log". 

## Other notes
This node should check the mode switch for when to go into MANUAL, or default to AUTO if it's not getting a receiver signal from the Spektrum receiver. 

## Interface control
For PT1, the expected messages on the nm2k bus are heartbeat, ISO address claim, and
Battery Status PGN: 127508
     Field 1: Battery instance
             2: Battery voltage
             3: Battery Current
             4: Battery Case temperature
             5: SID
Position, Rapid update: 129025
             1: Latitude
             2: Longitude
Direction Data: 130577
              1: Data Mode
              2: COG ref
              3: reserve bits
              4: SID
              5: COG
              6: SOG
              7: heading
              8: Speed through water
              9: set
             10: drift
			 
The bridge node has to put out heartbeat and ISO address claim but need NOT respond to the others for PT1. However, it must relay them to the RPi (generic nm2k frame capture). 

## Applicability
USNA Sailbot AY20 Hull 14 mod 3 and later.
