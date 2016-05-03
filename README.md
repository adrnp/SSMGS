# Super Simple Mavlink Ground Station #

This is a super simple "ground station" implemented in python using pymavlink that write the incoming GPS data to a KML file that can be viewed in Google Earth in real time.

## Dependencies ##

This code has the following dependencies:

 - pymavlink
 - simplekml
 - pyserial
 - 

There may be a couple that I have forgotten that aren't listed, but it will give you a heads up anyway.  All of the dependencies can be obtained simply with `pip install DEPENDENCY`.

## How to Use ##

### Starting the Script ###

To start the script, simply run:

	python groundstation.py --device=DEVICE

where `DEVICE` on Windows will be something like `COM1` and on Linux it will be something like `/dev/ttyUSB1` and Mac's are unnecessarily complicated here, so I'll let you figure out what it's called if you're using a Mac.

### To View the Map ###

simply open up the *balloon_link.kml* file in Google Earth and viola!