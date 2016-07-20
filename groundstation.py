""" groundstation.py: a simple mavlink groundstation that creates a KML file with the vehicle's current location """
import os
import simplekml
import csv
from pymavlink import mavutil
from argparse import ArgumentParser

__author__ = "Adrien Perkins"

'''
helper functions
'''


def wait_heartbeat(m):
    # wait for a heartbeat so we know the target system IDs
    print("Waiting for heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))


def wait_pos(m):
    # wait for a vehicle location
    # print("waiting for pos");
    msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg is not None:
        print("lat: %d, lon: %d, alt: %d" % (msg.lat, msg.lon, msg.alt))


def get_next_msg(m):
    # wait for the next message
    msg = m.recv_match(blocking=True)
    return msg


def create_network_link():
    # create a network link file for for the balloon
    linkKml = simplekml.Kml()
    networklink = linkKml.newnetworklink(name="Refresh Link")
    networklink.link.href = os.getcwd() + "/balloon.kml"
    networklink.link.refreshmode = simplekml.RefreshMode.oninterval
    networklink.link.refreshinterval = 1.0
    linkKml.save("balloon_link.kml")


'''
the code itself
'''

# set up the argument parser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--baudrate", type=int,
                    help="antenna baud rate", default=57600)
parser.add_argument("--device", required=True, help="serial device")
parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                    default=255, help='MAVLink source system for this GCS')
args = parser.parse_args()


# the logfile names
logfile_name = "all_logs.csv"

# create a mavlink serial instance
mavutil.set_dialect('common')
master = mavutil.mavlink_connection(args.device, baud=args.baudrate, source_system=args.SOURCE_SYSTEM)


# create a network link file
create_network_link()

# style to be used
styleGreen = simplekml.Style()
styleGreen.linestyle.color = simplekml.Color.green
styleGreen.polystyle.color = simplekml.Color.changealphaint(180, simplekml.Color.forestgreen)
styleGreen.linestyle.width = 2

# setup the ballon animation KML
balloonKml = simplekml.Kml()
balloonKml.document.name = "Balloon"

balloonPt = balloonKml.newpoint(name="balloon")
balloonPt.style.iconstyle.icon.href = "http://maps.google.com/mapfiles/kml/paddle/grn-blank.png"
balloonPt.style.iconstyle.heading = 0.0
balloonPt.altitudemode = simplekml.AltitudeMode.relativetoground
balloonPt.extrude = 1

balloonTrace = balloonKml.newlinestring(name="path")
balloonTrace.style = styleGreen
balloonTrace.altitudemode = simplekml.AltitudeMode.relativetoground
balloonTrace.extrude = 1

prevLocs = []
pathLength = 150  # how many points to save in the trace (if set to -1, then will add all points, this will generate a huge file)
haveFirstPos = False


# define all the variables here
lat = 0.0
lon = 0.0
alt = 0.0

print "starting loop"
pathUpdated = False
with open(logfile_name, 'ab') as f:

    writer = csv.writer(f, delimiter=",")
    writer.writerow(['lat', 'lon', 'alt', 'rssi', 'remrssi', 'noise', 'remnoise'])

    while True:

        # get the nextr mavlink message
        msg = get_next_msg(master)

        # get the attitude (nto configured yet)
        if msg.get_type() == "ATTITUDE":
            att = msg
            print("roll: %f, pitch: %f, yaw: %f" % (att.roll, att.pitch, att.yaw))

        # get the radio status
        elif msg.get_type() == "RADIO_STATUS":
            status = msg
            localdBm = (status.rssi / 1.9) - 127
            localNoisedBm = (status.noise / 1.9) - 127
            remdBm = (status.remrssi / 1.9) - 127
            remNoisedBm = (status.remnoise / 1.9) - 127

            localFadeMargin = localdBm - localNoisedBm
            remFadeMargin = remdBm - remNoisedBm

            distMultipler = 2**(localFadeMargin/6)

            balloonPt.name = str(localdBm)
            pathUpdated = True

            writer.writerow([lat, lon, alt, localdBm, remdBm, localNoisedBm, remNoisedBm])

            print("rssi: %f dBm, remrssi: %f dBm, noise: %f dBm, remnoise: %f dBm, rxerrors: %f\n" % (localdBm, remdBm, localNoisedBm, remNoisedBm, status.rxerrors))
            print("local fade margin: %f dB, remote fade margin: %f dB, distance multiplier: %f\n" % (localFadeMargin, remFadeMargin, distMultipler))

        # get the heartbeat (doing this to be able to send a heartbeat)
        elif msg.get_type() == "HEARTBEAT":
            hrt = msg
            print("heartbeat received: base: %d,  custom: %d, state: %d\n" % (hrt.base_mode, hrt.custom_mode, hrt.system_status))

            # send a heartbeat if received one
            master.mav.heartbeat_send(1, 1, 1, 1, 1)

        # the raw gps values (not sure this is used...)
        elif msg.get_type() == "GPS_RAW_INT":
            gps = msg
            lat = gps.lat / 10000000.
            lon = gps.lon / 10000000.
            alt = gps.alt / 1000.
            print("gps raw lat: %f, lon: %f, alt: %d" % (lat, lon, alt))

            if len(prevLocs) < pathLength:
                prevLocs.append((lon, lat, alt))
            else:
                prevLocs.pop(0)
                prevLocs.append((lon, lat, alt))

            # update the plane kml file
            balloonPt.coords = [(lon, lat, alt)]
            balloonTrace.coords = prevLocs

            pathUpdated = True

        # update the KML file as needed
        if pathUpdated:
            pathUpdated = False
            balloonKml.save("balloon.kml")
