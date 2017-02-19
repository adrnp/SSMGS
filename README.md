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


## Custom Mavlink Protocol ##

If you want to use a custom dialect for Mavlink (such as being done here) there is one additional necessary step to be able to use that dialect.  In this case all the definitions can be found in the `ncl_ground.py` file.

To use the `ncl_ground` dialect, we need to place the `ncl_ground.py` file into pymavlink's dialect folder on your machine.  For a Windows machine with python installed via Anaconda, this can be found at `C:\Users\USERNAME\Anaconda2\Lib\site-packages\pymavlink\dialects\v10`.

Simply copy and paste `ncl_ground.py` into that folder and then you'll be good to go!