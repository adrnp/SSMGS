"""
TKinter based ground station
"""

import Tkinter
import time
import threading
import random
import Queue

import os
import simplekml
import csv
from pymavlink import mavutil
from argparse import ArgumentParser


# list of message types that will be used
MSG_TYPE_RADIO = 1
MSG_TYPE_HEARTBEAT = 2
MSG_TYPE_PRESSURE = 3
MSG_TYPE_GPS = 4
MSG_TYPE_TEMPERATURE = 5
MSG_TYPE_COMMAND_ACK = 6


class GuiPart:
    def __init__(self, master, queue, endCommand):
        self.queue = queue
        # Set up the GUI
        #console = Tkinter.Button(master, text='Done', command=endCommand)
        #console.pack()

        #T = Tkinter.Text(master, height=2, width=30)
        #T.pack()
        #T.insert(1.0, "Just a text Widget\nin two lines\n")

        self.main_container = Tkinter.Frame(master, background="bisque")
        self.main_container.pack(side="top", fill="both", expand=True)

        self.defaultbg = master.cget('bg')

        self.top_frame = Tkinter.Frame(self.main_container)
        self.main_frame = Tkinter.Frame(self.main_container, background="yellow")
        self.base_frame = Tkinter.Frame(self.main_container)
        self.top_frame.pack(side="top", fill="x", expand=False)
        self.base_frame.pack(side="bottom", fill="x", expand=False)
        self.main_frame.pack(side="bottom", fill="both", expand=True)

        # the button at the bottom
        console = Tkinter.Button(self.base_frame, text='Done', command=endCommand)
        console.pack()
        self.unit_button = Tkinter.Button(self.base_frame, text="Meters", command=self.change_unit)
        self.unit_dist_mult = 1.0
        self.unit_text = "m"
        self.unit_button.pack();
        
        # top bar with the valid states
        self.state_ascent = Tkinter.Label(self.top_frame, text="Ascent", bg="green")
        self.state_ascent.pack(padx=5, pady=10, side="left")

        self.state_burst = Tkinter.Label(self.top_frame, text="Burst")
        self.state_burst.pack(padx=5, pady=10, side="left")

        self.state_free_fall = Tkinter.Label(self.top_frame, text="Free Fall")
        self.state_free_fall.pack(padx=5, pady=10, side="left")

        self.state_descent = Tkinter.Label(self.top_frame, text="Descent")
        self.state_descent.pack(padx=5, pady=10, side="left")

        # mission time
        left_mission_time = Tkinter.Frame(self.main_frame)
        left_mission_time.pack(side="top",fill="x", expand=True)
        mission_time_name = Tkinter.Label(left_mission_time, text="Mission Time: ")
        mission_time_name.pack(padx=5, pady=10, side="left")

        self.mission_time = Tkinter.Label(left_mission_time, text="0")
        self.mission_time.pack(padx=0, pady=10, side="left")

        # the heater information
        left_heater = Tkinter.Frame(self.main_frame)
        left_heater.pack(side="top",fill="x", expand=True)
        heater_title = Tkinter.Label(left_heater, text="Heater Information")
        heater_title.pack(side="top")
        self.heater12 = Tkinter.Label(left_heater, text="heater 12")
        self.heater12.pack(padx=5, pady=10, side="left")

        self.heater34 = Tkinter.Label(left_heater, text="heater 34")
        self.heater34.pack(padx=5, pady=10, side="left")

        self.heater56 = Tkinter.Label(left_heater, text="heater 56")
        self.heater56.pack(padx=5, pady=10, side="left")

        
        # temp data
        left_temp = Tkinter.Frame(self.main_frame)
        left_temp.pack(side="top",fill="x", expand=True)
        sensor_title = Tkinter.Label(left_temp, text="Sensor Info")
        sensor_title.pack(side="top")
        temp_name = Tkinter.Label(left_temp, text="Temperature: ")
        temp_name.pack(padx=5, pady=5, side="left")

        self.temp = Tkinter.Label(left_temp, text="0")
        self.temp.pack(padx=0, pady=5, side="left")

        # pressure data
        left_pressure = Tkinter.Frame(self.main_frame)
        left_pressure.pack(side="top",fill="x", expand=True)
        pressure_name = Tkinter.Label(left_pressure, text="Pressure: ")
        pressure_name.pack(padx=5, pady=5, side="left")

        self.pressure = Tkinter.Label(left_pressure, text="0")
        self.pressure.pack(padx=0, pady=5, side="left")

        # baro alt
        left_baroalt = Tkinter.Frame(self.main_frame)
        left_baroalt.pack(side="top",fill="x", expand=True)
        baroalt_name = Tkinter.Label(left_baroalt, text="Baro Alt: ")
        baroalt_name.pack(padx=5, pady=5, side="left")

        self.baro_alt = Tkinter.Label(left_baroalt, text="0")
        self.baro_alt.pack(padx=0, pady=5, side="left")

        # gps alt
        left_gpsalt = Tkinter.Frame(self.main_frame)
        left_gpsalt.pack(side="top",fill="x", expand=True)
        gpsalt_name = Tkinter.Label(left_gpsalt, text="GPS Alt: ")
        gpsalt_name.pack(padx=5, pady=5, side="left")

        self.gps_alt = Tkinter.Label(left_gpsalt, text="0")
        self.gps_alt.pack(padx=0, pady=5, side="left")

        # radio data
        left_radio1 = Tkinter.Frame(self.main_frame)
        left_radio1.pack(side="top",fill="x", expand=True)
        radio_title = Tkinter.Label(left_radio1, text="Radio Info")
        radio_title.pack(side="top")
        rssi_name = Tkinter.Label(left_radio1, text="RSSI: ")
        rssi_name.pack(padx=5, pady=2, side="left")

        self.rssi = Tkinter.Label(left_radio1, text="0")
        self.rssi.pack(padx=0, pady=2, side="left")

        remrssi_name = Tkinter.Label(left_radio1, text="Remote RSSI: ")
        remrssi_name.pack(padx=5, pady=2, side="left")

        self.remote_rssi = Tkinter.Label(left_radio1, text="0")
        self.remote_rssi.pack(padx=0, pady=2, side="left")

        # radio noise
        left_radio2 = Tkinter.Frame(self.main_frame)
        left_radio2.pack(side="top",fill="x", expand=True)
        noise_name = Tkinter.Label(left_radio2, text="Noise: ")
        noise_name.pack(padx=5, pady=5, side="left")

        self.noise = Tkinter.Label(left_radio2, text="0")
        self.noise.pack(padx=0, pady=5, side="left")

        remnoise_name = Tkinter.Label(left_radio2, text="Remote Noise: ")
        remnoise_name.pack(padx=5, pady=5, side="left")

        self.remote_noise = Tkinter.Label(left_radio2, text="0")
        self.remote_noise.pack(padx=0, pady=5, side="left")

        # radio fade margin
        left_radio3 = Tkinter.Frame(self.main_frame)
        left_radio3.pack(side="top",fill="x", expand=True)
        fm_name = Tkinter.Label(left_radio3, text="Fade Margin: ")
        fm_name.pack(padx=5, pady=5, side="left")

        self.fademargin = Tkinter.Label(left_radio3, text="0")
        self.fademargin.pack(padx=0, pady=5, side="left")

        remfm_name = Tkinter.Label(left_radio3, text="Remote Fade Margin: ")
        remfm_name.pack(padx=5, pady=5, side="left")

        self.remote_fademargin = Tkinter.Label(left_radio3, text="0")
        self.remote_fademargin.pack(padx=0, pady=5, side="left")

        # radio distance multiplier
        left_radio4 = Tkinter.Frame(self.main_frame)
        left_radio4.pack(side="top",fill="x", expand=True)
        dist_mult_name = Tkinter.Label(left_radio4, text="Dist Mult: ")
        dist_mult_name.pack(padx=5, pady=5, side="left")

        self.dist_mult = Tkinter.Label(left_radio4, text="0")
        self.dist_mult.pack(padx=0, pady=5, side="left")


        # Add more GUI stuff here

    def processIncoming(self):
        """
        Handle all the messages currently in the queue (if any).
        """
        while self.queue.qsize():
            try:
                msg = self.queue.get(0)

                if msg['type'] == MSG_TYPE_HEARTBEAT:  # heartbeat message

                    if msg['heater1']:
                        self.heater12.config(bg="green")
                    else:
                        self.heater12.config(bg=self.defaultbg)

                    if msg['heater2']:
                        self.heater34.config(bg="green")
                    else:
                        self.heater34.config(bg=self.defaultbg)

                    if msg['heater3']:
                        self.heater56.config(bg="green")
                    else:
                        self.heater56.config(bg=self.defaultbg)

                    if msg['state'] == "Ascent":
                        self.state_ascent.config(bg="green")
                        self.state_burst.config(bg=self.defaultbg)
                        self.state_free_fall.config(bg=self.defaultbg)
                        self.state_descent.config(bg=self.defaultbg)
                    elif msg['state'] == "Burst":
                        self.state_ascent.config(bg=self.defaultbg)
                        self.state_burst.config(bg="green")
                        self.state_free_fall.config(bg=self.defaultbg)
                        self.state_descent.config(bg=self.defaultbg)
                    elif msg['state'] == "Free Fall":
                        self.state_ascent.config(bg=self.defaultbg)
                        self.state_burst.config(bg=self.defaultbg)
                        self.state_free_fall.config(bg="green")
                        self.state_descent.config(bg=self.defaultbg)
                    elif msg['state'] == "Descent":
                        self.state_ascent.config(bg=self.defaultbg)
                        self.state_burst.config(bg=self.defaultbg)
                        self.state_free_fall.config(bg=self.defaultbg)
                        self.state_descent.config(bg="green")


                elif msg['type'] == MSG_TYPE_RADIO:  # radio status

                    self.rssi.config(text='{:04.2f}'.format(msg['rssi']))
                    self.remote_rssi.config(text='{:04.2f}'.format(msg['remote_rssi']))
                    self.noise.config(text='{:04.2f}'.format(msg['noise']))
                    self.remote_noise.config(text='{:04.2f}'.format(msg['remote_noise']))
                    self.fademargin.config(text='{:04.2f}'.format(msg['fade_margin']))
                    self.remote_fademargin.config(text='{:04.2f}'.format(msg['remote_fade_margin']))
                    self.dist_mult.config(text='{:04.2f}'.format(msg['dist_mult']))

                elif msg['type'] == MSG_TYPE_PRESSURE:  # temp pressure sensor

                    self.mission_time.config(text='{:04.2f}'.format(msg['mission_time']))
                    self.baro_alt.config(text='{:04.2f} {}'.format(msg['baro_altitude']*self.unit_dist_mult, self.unit_text))
                    self.pressure.config(text='{:04.2f} Pa'.format(msg['pressure']))

                elif msg['type'] == MSG_TYPE_GPS:  # gps altitude
                    self.gps_alt.config(text='{:04.2f} {}'.format(msg['gps_alt']*self.unit_dist_mult, self.unit_text))

                elif msg['type'] == MSG_TYPE_TEMPERATURE:
                    self.temp.config(text='{:04.2f} C'.format(msg['board_temperature']))
                    #TODO: add all of the other temps

            except Queue.Empty:
                pass

    def change_unit(self):
        if self.unit_dist_mult == 1.0:  # currently in meters
            self.unit_dist_mult = 3.28  # feet in a meter
            self.unit_text = "ft"
            self.unit_button.config(text="Feet")
        else:
            self.unit_dist_mult = 1.0
            self.unit_text = "m"
            self.unit_button.config(text="Meters")


class ThreadedClient:
    """
    Launch the main part of the GUI and the worker thread. periodicCall and
    endApplication could reside in the GUI part, but putting them here
    means that you have all the thread controls in a single place.
    """
    def __init__(self, master, device, baudrate, source_system):
        """
        Start the GUI and the asynchronous threads. We are in the main
        (original) thread of the application, which will later be used by
        the GUI. We spawn a new thread for the worker.
        """
        self.master = master
        self.device = device
        self.baudrate = baudrate
        self.source_system = source_system

        # Create the queue
        self.queue = Queue.Queue()

        # Set up the GUI part
        self.gui = GuiPart(master, self.queue, self.endApplication)

        # Set up the thread to do asynchronous I/O
        # More can be made if necessary
        self.running = 1
    	self.thread1 = threading.Thread(target=self.workerThread1)
        self.thread1.start()

        # Start the periodic call in the GUI to check if the queue contains
        # anything
        self.periodicCall()

    def periodicCall(self):
        """
        Check every 100 ms if there is something new in the queue.
        """
        self.gui.processIncoming()
        if not self.running:
            # This is the brutal stop of the system. You may want to do
            # some cleanup before actually shutting it down.
            import sys
            sys.exit(1)
        self.master.after(100, self.periodicCall)


    def get_next_msg(self, m):
        # wait for the next message
        msg = m.recv_match(blocking=True)
        return msg


    def create_network_link(self):
        # create a network link file for for the balloon
        linkKml = simplekml.Kml()
        networklink = linkKml.newnetworklink(name="Refresh Link")
        networklink.link.href = os.getcwd() + "/balloon.kml"
        networklink.link.refreshmode = simplekml.RefreshMode.oninterval
        networklink.link.refreshinterval = 1.0
        linkKml.save("balloon_link.kml")

    def parseRadioMsg(self, msg):

        returnMsg = {}

        # get the radio status
        if msg.get_type() == "RADIO_STATUS":
            status = msg
            localdBm = (status.rssi / 1.9) - 127
            localNoisedBm = (status.noise / 1.9) - 127
            remdBm = (status.remrssi / 1.9) - 127
            remNoisedBm = (status.remnoise / 1.9) - 127

            localFadeMargin = localdBm - localNoisedBm
            remFadeMargin = remdBm - remNoisedBm

            distMultipler = 2**(localFadeMargin/6)

            #balloonPt.name = str(localdBm)
            #pathUpdated = True

            #writer.writerow([lat, lon, alt, localdBm, remdBm, localNoisedBm, remNoisedBm])

            returnMsg['type'] = MSG_TYPE_RADIO
            returnMsg['rssi'] = localdBm
            returnMsg['remote_rssi'] = remdBm
            returnMsg['noise'] = localNoisedBm
            returnMsg['remote_noise'] = remNoisedBm
            returnMsg['fade_margin'] = localFadeMargin
            returnMsg['remote_fade_margin'] = remFadeMargin
            returnMsg['dist_mult'] = distMultipler

            print("rssi: %f dBm, remrssi: %f dBm, noise: %f dBm, remnoise: %f dBm, rxerrors: %f\n" % (localdBm, remdBm, localNoisedBm, remNoisedBm, status.rxerrors))
            print("local fade margin: %f dB, remote fade margin: %f dB, distance multiplier: %f\n" % (localFadeMargin, remFadeMargin, distMultipler))

        # get the heartbeat (doing this to be able to send a heartbeat)
        elif msg.get_type() == "HEARTBEAT":
            hrt = msg

            # all of the flags
            flightTermFlag = False
            flightTermBurnStart = False
            flightTermBurnEnd = False
            parachuteArmed = False
            parachuteDeployed = False
            heatOn = False
            heatPriority = False

            # parse out the base mode elements
            baseMode = hrt.base_mode
            if baseMode & 1 > 0:
                flightTermination = True

            if baseMode & 2 > 0:
                flightTermBurnStart = True

            if baseMode & 4 > 0:
                flightTermBurnEnd = True

            if baseMode & 8 > 0:
                parachuteArmed = True

            if baseMode & 16 > 0:
                parachuteDeployed = True

            if baseMode & 32 > 0:
                heatOn = True

            if baseMode & 64 > 0:
                heatPriority = True


            # heater state
            heater1on = False
            heater2on = False
            heater3on = False
            nichromeon = False

            customMode = hrt.custom_mode
            if customMode & 1 > 0:
                heater1on = True

            if customMode & 2 > 0:
                heater2on = True

            if customMode & 4 > 0:
                heater3on = True

            if customMode & 65536 > 0:
                nichromeon = True

            state = "unknown"
            sysState = hrt.system_status
            if sysState == 0:
                state = "Ascent"
            elif sysState == 1:
                state = "Burst"
            elif sysState == 2:
                state = "Free Fall"
            elif sysState == 3:
                state = "Descent"

            print("heartbeat received: base: %d,  custom: %d, state: %d\n" % (hrt.base_mode, hrt.custom_mode, hrt.system_status))

            returnMsg['type'] = MSG_TYPE_HEARTBEAT
            returnMsg['heater1'] = heater1on
            returnMsg['heater2'] = heater2on
            returnMsg['heater3'] = heater3on
            returnMsg['state'] = state

            
        elif msg.get_type() == "TEMP_SENSORS":

            timestamp = msg.time_usec  # I don't think this is really needed
            tempArray = msg.temperature
            boardTemp = msg.board_temperature
            print("temp received: board temp: %f\n" % (boardTemp))

            returnMsg['type'] = MSG_TYPE_TEMPERATURE
            returnMsg['temperature_array'] = tempArray
            returnMsg['board_temperature'] = boardTemp

        elif msg.get_type() == "PRESSURE_SENSOR":

            missionTime = msg.mission_time
            baroAlt = msg.baro_altitude
            pressure = msg.pressure
            print("sensor received: mission time: %d, baro alt: %f, pressure: %f\n" % (missionTime, baroAlt, pressure))

            returnMsg['type'] = MSG_TYPE_PRESSURE
            returnMsg['mission_time'] = missionTime/1000/60;  # convert milliseconds to minutes
            returnMsg['baro_altitude'] = baroAlt
            returnMsg['pressure'] = pressure

        elif msg.get_type() == "COMMAND_ACK":

            res = msg.result
            print("received ack of the nichrome command\n")

            returnMsg['type'] = MSG_TYPE_COMMAND_ACK

        return returnMsg


    def workerThread1(self):
        """
        This is where we handle the asynchronous I/O. For example, it may be
        a 'select()'.
        One important thing to remember is that the thread has to yield
        control.
        """

        
        # create a mavlink serial instance
        mavutil.set_dialect('ncl_ground')
        master = mavutil.mavlink_connection(self.device, baud=self.baudrate, source_system=self.source_system)


        # create a network link file
        self.create_network_link()

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
        

        while self.running:

            # get the nextr mavlink message
            msg = self.get_next_msg(master)

            # the raw gps values (not sure this is used...)
            if msg.get_type() == "GPS_RAW_INT":
                gps = msg
                lat = gps.lat / 10000000.
                lon = gps.lon / 10000000.
                alt = gps.alt / 1000.
                print("gps raw lat: %f, lon: %f, alt: %d\n" % (lat, lon, alt))

                if len(prevLocs) < pathLength:
                    prevLocs.append((lon, lat, alt))
                else:
                    prevLocs.pop(0)
                    prevLocs.append((lon, lat, alt))

                # update the plane kml file
                balloonPt.coords = [(lon, lat, alt)]
                balloonTrace.coords = prevLocs

                balloonKml.save("balloon.kml")

                toSend = {}
                toSend['type'] = MSG_TYPE_GPS
                toSend['gps_alt'] = alt
                self.queue.put(toSend)

            else:
                toSend = self.parseRadioMsg(msg)
                if toSend:
                    self.queue.put(toSend)

            if msg.get_type() == "HEARTBEAT":
                # send a heartbeat if received one
                master.mav.heartbeat_send(2, 2, 2, 2, 2)


    def endApplication(self):
        self.running = 0


# set up the argument parser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--baudrate", type=int,
                    help="antenna baud rate", default=57600)
parser.add_argument("--device", required=True, help="serial device")
parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                    default=250, help='MAVLink source system for this GCS')
args = parser.parse_args()


rand = random.Random()
root = Tkinter.Tk()

client = ThreadedClient(root, args.device, args.baudrate, args.SOURCE_SYSTEM)
root.mainloop()