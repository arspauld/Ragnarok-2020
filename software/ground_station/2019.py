import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph import console
from pyqtgraph.dockarea import *
from realplot import RealTimePlot, update
import serial
import string

#  Open the serial port
ser = serial.Serial()
ser.timeout = 0.02
ser.baudrate = 115200
ser.port = 'COM3'
ser.open()

#comment

#  Commands
"""For use with the ground station to control the GUI"""
RESET = b'f'
CALIBRATE_PAYLOAD = b'e'
REQUEST_PACKET = b'd'
CALIBRATE_ALTITUDE = b'c'
CALIBRATE_ANGLE = b'b'
SERVO_RELEASE = b'a'
SERVO_CLOSE = b's'


#GUI Setup
app = QtGui.QApplication([])                    # Starts the GUI application
win = QtGui.QMainWindow()                       # Creates the main storage for docks
area = DockArea()                               # Creates an area to dock into
win.setCentralWidget(area)                      # Adds the area to the window space
win.resize(1400,800)                            # Sets the window size to 1400 pixels wide by 800 pixels tall
win.setWindowTitle("Skyfire Ground Station - Team 5343")    # Names the window

## Creates Docks and inserts them into the area
altituded  = Dock("Altitude")       # Check realplot.py for documentation
pressured  = Dock("Pressure")       # Creates plotting elements for each telemetry section
tempd      = Dock("Temp")
voltaged   = Dock("Voltage")
pitchd     = Dock("Pitch")
rolld      = Dock("Roll")
spind      = Dock("Blade Spin Rate")
directiond = Dock("Bonus Direction")
gpsd       = Dock("GPS")
messages   = Dock("Communication")
area.addDock(altituded, 'left')
area.addDock(pressured, 'bottom', altituded)
area.addDock(voltaged, 'bottom', pressured)
area.addDock(tempd, 'right', altituded)
area.addDock(gpsd, 'right', pressured)
area.addDock(pitchd, 'right', voltaged)
area.addDock(directiond, 'right', tempd)
area.addDock(spind, 'right', gpsd)
area.addDock(rolld, 'right', pitchd)
area.addDock(messages, 'right')

## Add Widgets to the docks
# Plots
wid = QtGui.QWidget()                           # Creates an empty Widget to hold a layout
lay = QtGui.QGridLayout()                       # Creates a layout to orgonaize the plots
wid.setLayout(lay)                              # Adds the layout to the container widget

altitude  = RealTimePlot(name="Altitude (m)")       # Check realplot.py for documentation
pressure  = RealTimePlot(name="Pressure (Pa)")       # Creates plotting elements for each telemetry section
temp      = RealTimePlot(name="Temp (°C)")
voltage   = RealTimePlot(name="Voltage (V)")
pitch     = RealTimePlot(name="Pitch (°)")
roll      = RealTimePlot(name="Roll (°)")
spin      = RealTimePlot(name="Blade Spin Rate (rpm)")
direction = RealTimePlot(name="Bonus Direction (°)")
gps       = RealTimePlot(name="GPS (°)")
altituded.addWidget(altitude.plot)          # Adds the widgets to the layout
pressured.addWidget(pressure.plot)
tempd.addWidget(temp.plot)
voltaged.addWidget(voltage.plot)
pitchd.addWidget(pitch.plot) 
rolld.addWidget(roll.plot)
gpsd.addWidget(gps.plot)
spind.addWidget(spin.plot)
directiond.addWidget(direction.plot)


# Buttons
w = QtGui.QWidget()                                                 # Creates a large widget to hold the others

halt_btn = QtGui.QPushButton('HALT')                                # A pushable button
calibrate_btn = QtGui.QPushButton('Calibrate Payload')
packet_btn = QtGui.QPushButton('Request Packet')
calibrate_altitude_btn = QtGui.QPushButton('Calibrate Altitude')
calibrate_angle_btn = QtGui.QPushButton('Calibrate Angle')
gps_btn = QtGui.QPushButton('GPS')
reset_btn = QtGui.QPushButton('RESET')
servo_release_btn = QtGui.QPushButton('Servo Release')
servo_close_btn = QtGui.QPushButton('Servo Close')

halt_btn.setCheckable(True)                          # Allows for a button to stay pressed
calibrate_btn.setCheckable(True)
packet_btn.setCheckable(True)
calibrate_altitude_btn.setCheckable(True)
calibrate_angle_btn.setCheckable(True)
gps_btn.setCheckable(True)
reset_btn.setCheckable(True)
servo_release_btn.setCheckable(True)
servo_close_btn.setCheckable(True)

listw = QtGui.QListWidget()                     # Creates a display list box
cmdw = QtGui.QLineEdit()                        # Creates a line editor
mission_timew = QtGui.QLabel()                  # Creates a text label
packetsw = QtGui.QLabel()
flight_statew = QtGui.QLabel()
gps_timew = QtGui.QLabel()
gps_altw = QtGui.QLabel()
gps_satsw = QtGui.QLabel()

mission_timew.setText('<b>Mission Time (s):</b>')       # Set the initial text of the labels to the data they reperesent
packetsw.setText('<b>Packets:</b>')
flight_statew.setText('<b>Flight State:</b>')
gps_timew.setText('<b>GPS Time (UTC):</b>')
gps_altw.setText('<b>GPS Altitude (m):</b>')
gps_satsw.setText('<b>GPS Sats:</b>')

layout2 = QtGui.QGridLayout()                   # Creates a layout element
w.setLayout(layout2)                            # Adds the layout to the large widget
messages.addWidget(w)                           # Adds the large widget to the dock

layout2.addWidget(reset_btn,                7, 0, 1, 2)     # Widget goes in row 7, column 0, spanning 1 row and 2 columns
layout2.addWidget(halt_btn,                 8, 0, 1, 1)
layout2.addWidget(listw,                    0, 0, 1, 2)
layout2.addWidget(cmdw,                     1, 0, 1, 2)
layout2.addWidget(packet_btn,               9, 0, 1, 1)
layout2.addWidget(calibrate_altitude_btn,   9, 1, 1, 1)
layout2.addWidget(calibrate_angle_btn,      8, 1, 1, 1)
layout2.addWidget(calibrate_btn,            10, 1, 1, 1)
layout2.addWidget(gps_btn,                  10, 0, 1, 1)
layout2.addWidget(servo_release_btn,        11, 0, 1, 1)
layout2.addWidget(servo_close_btn,          11, 1, 1, 1)
layout2.addWidget(mission_timew,            3, 0, 1, 1)
layout2.addWidget(packetsw,                 4, 0, 1, 1)
layout2.addWidget(flight_statew,            5, 0, 1, 1)
layout2.addWidget(gps_timew,                3, 1, 1, 1)
layout2.addWidget(gps_altw,                 4, 1, 1, 1)
layout2.addWidget(gps_satsw,                5, 1, 1, 1)

# Show Window
win.show()  

#GUI Functions
def addItem():
    command = cmdw.text()

    if command.lower() == 'gclear':
        altitude.clear()
        pressure.clear()
        temp.clear()
        voltage.clear()
        pitch.clear()
        roll.clear()
        spin.clear()
        direction.clear()
        gps.clear()

        listw.addItem('gclear')
        cmdw.setText('')

    elif command.lower() == 'cclear':
        listw.clear()

        listw.addItem('cclear')
        cmdw.setText('')

    elif command.lower() == 'clearall':
        reset()

        listw.addItem('clearall')
        cmdw.setText('')

    elif command.lower() == 'servo_release':
        ser.write(b'a')

        listw.addItem('SERVO_RELEASE')
        cmdw.setText('')

    elif command.lower() == 'servo_close':
        ser.write(b's')

        listw.addItem('SERVO_CLOSE')
        cmdw.setText('')

    elif command.lower() == 'reset':
        ser.write(b'f')

        reset()
        listw.addItem('RESET')
        cmdw.setText('')

    elif command.lower() == 'calibrate_payload':
        ser.write(b'e')

        altitude.clear()
        pitch.clear()
        roll.clear()

        listw.addItem('CALIBRATE_PAYLOAD')
        cmdw.setText('')

    elif command.lower() == 'calibrate_altitude':
        ser.write(b'c')

        altitude.clear()
        listw.addItem('CALIBRATE_ALTITUDE')
        cmdw.setText('')

    elif command.lower() == 'calibrate_angle':
        ser.write(b'b')

        pitch.clear()
        roll.clear()

        listw.addItem('CALIBRATE_ANGLE')
        cmdw.setText('')

    elif command.lower() == 'request_packet':
        ser.write(b'd')

        listw.addItem('REQUEST_PACKET')
        cmdw.setText('')

    else:
        listw.addItem(command)
        cmdw.setText('')
    
def write2payload():
    text = cmdw.text()
    binary = text.encode('ascii')
    ser.write(binary)

#Button Functions
def halt():
    if halt_btn.isChecked():
        timer.stop()
    else:
        timer.start()
    listw.addItem('HALT')

def reset():
    altitude.clear()
    pressure.clear()
    temp.clear()
    voltage.clear()
    pitch.clear()
    roll.clear()
    spin.clear()
    direction.clear()
    gps.clear()

    listw.clear()

    if reset_btn.isChecked():
        reset_btn.toggle()
        listw.addItem('RESET')
        ser.write(b'f')

def calibrate():
    altitude.clear()
    pitch.clear()
    roll.clear()
    direction.clear()

    calibrate_btn.toggle()

    listw.addItem('CALIBRATE_PAYLOAD')
    ser.write(b'e')

def request_packet():
    parse_serial()
    
    packet_btn.toggle()

    listw.addItem('REQUEST_PACKET')
    ser.write(b'd')

def calibrate_altitude():
    altitude.clear()
    
    calibrate_altitude_btn.toggle()

    listw.addItem('CALIBRATE_ALTITUDE')
    ser.write(b'c')

def calibrate_angle():
    pitch.clear()
    roll.clear()
    direction.clear()
    
    calibrate_angle_btn.toggle()

    listw.addItem('CALIBRATE_ANGLE')
    ser.write(b'b')

def get_gps():
    gps_btn.toggle()

    listw.addItem('GET_GPS')
    listw.addItem(str(datapoints[9]) + ',' + str(datapoints[8]))

def servo_release():
    servo_release_btn.toggle()

    listw.addItem('SERVO_RELEASE')
    ser.write(b'a')

def servo_close():
    servo_close_btn.toggle()

    listw.addItem('SERVO_CLOSE')
    ser.write(b's')

#Write to .csv file
def csv_write(line):
    z = open('Flight_5343.csv','a')
    line.split(',')
    z.write(line)
    z.close()

headers = 'Team ID,Mission Time,Packet Count,Altitude,Pressure,Temperature,Voltage,GPS Time,GPS Latitude,GPS Longitude,GPS Altitude,GPS Sats,Pitch,Roll,Blade Spin Rate,Flight State,Bonus Direction\n'

w = open('Flight_5343.csv','w+')

csv_write(headers)

w.close()

#Graphing and Data Functions
def graph(datapoints):
    altitude.add(       float(datapoints[1]),  float(datapoints[3]))
    pressure.add(       float(datapoints[1]),  float(datapoints[4]))
    temp.add(           float(datapoints[1]),  float(datapoints[5]))
    voltage.add(        float(datapoints[1]),  float(datapoints[6]))
    pitch.add(          float(datapoints[1]),  float(datapoints[12]))
    roll.add(           float(datapoints[1]),  float(datapoints[13]))
    spin.add(           float(datapoints[1]),  float(datapoints[14]))
    direction.add(      float(datapoints[1]),  float(datapoints[16]))
    if float(datapoints[8]) != 0 and float(datapoints[9]) != 0:
        gps.add(            float(datapoints[9]),  float(datapoints[8]))

def parse_serial():
    binary = ser.readline()
    lines = str(binary, encoding='ascii')
    
    line = lines.strip()

    datapoints = line.split(',') #This is the actual data that can be written into the .csv file

    if datapoints[0] == '5343' or len(datapoints) == 2:

        listw.addItem(line)

    if datapoints[0] == '5343' and len(datapoints) > 2:
        csv_write(lines)

        graph(datapoints)
        mission_timew.setText('<b>Mission Time (s): </b>' + str(datapoints[1]))
        packetsw.setText('<b>Packets: </b>' + str(datapoints[2]))
        flight_statew.setText('<b>Flight State: </b>' + str(datapoints[15]))
        gps_timew.setText('<b>GPS Time (UTC): </b>' + str(datapoints[7]))
        gps_altw.setText('<b>GPS Altitude (m): </b>' + str(datapoints[10]))
        gps_satsw.setText('<b>GPS Sats: </b>' + str(datapoints[11]))


#Timer
timer = QtCore.QTimer()
timer.setInterval(100)
timer.timeout.connect(parse_serial)
timer.start()


#Connect Signals with Events (Functions)
cmdw.returnPressed.connect(write2payload)                       # Connects each event to a function
cmdw.returnPressed.connect(addItem)
halt_btn.clicked.connect(halt)
calibrate_btn.clicked.connect(calibrate)
packet_btn.clicked.connect(request_packet)
calibrate_altitude_btn.clicked.connect(calibrate_altitude)
calibrate_angle_btn.clicked.connect(calibrate_angle)
gps_btn.clicked.connect(get_gps)
reset_btn.clicked.connect(reset)
servo_release_btn.clicked.connect(servo_release)
servo_close_btn.clicked.connect(servo_close)





#Execute the GUI application
app.exec_()

#Close the serial port
ser.close()