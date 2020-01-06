import realplot
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph import console
from pyqtgraph.dockarea import *
import serial
import string
from PyQt5 import QtWidgets

class groundStation:
    def __init__(self, title, width, height):
        # Initializes the application with a centralized dock area containing all plots, buttons, lists, labels, and texts
        self.app = QtGui.QApplication([])
        self.window = QtGui.QMainWindow()
        self.area = DockArea()
        self.window.setCentralWidget(self.area)
        self.window.resize(width, height)
        self.window.setWindowTitle(title.title())
        """
        self.alt_dock   = Dock("Altitude")
        self.pres_dock  = Dock("Pressure")
        self.temp_dock  = Dock("Temperature")
        self.volt_dock  = Dock("Voltage")
        self.gps_dock   = Dock("GPS")
        self.airsp_dock = Dock("Airspeed")
        self.part_dock  = Dock("Particle Count")
        self.msg_dock   = Dock("Messages")

        self.area.addDock(self.alt_dock,     "left")
        self.area.addDock(self.pres_dock,    "bottom",   self.alt_dock)
        self.area.addDock(self.temp_dock,    "right",    self.alt_dock)
        self.area.addDock(self.volt_dock,    "right",    self.temp_dock)
        self.area.addDock(self.gps_dock,     "bottom",   self.temp_dock)
        self.area.addDock(self.airsp_dock,   "right",    self.gps_dock)
        self.area.addDock(self.part_dock,    "bottom",   self.pres_dock)
        self.area.addDock(self.msg_dock,     "right")

        self.widget = QtGui.QWidget()
        self.layout = QtGui.QGridLayout()
        self.widget.setLayout(self.layout)

        self.alt =   realplot.RTP(name = "Altitude (m)")
        self.pres =  realplot.RTP(name = "Pressure (Pa)")
        self.temp =  realplot.RTP(name = "Temperature (°C)")
        self.volt =  realplot.RTP(name = "Voltage (V)")
        self.gps =   realplot.RTP(name = "GPS (°)")
        self.airsp = realplot.RTP(name = "Airspeed (m/s)")
        self.part =  realplot.RTP(name = "Particle Count (ug/m^3)")

        self.alt_dock.addWidget(    self.alt.plot)
        self.pres_dock.addWidget(   self.pres.plot)
        self.temp_dock.addWidget(   self.temp.plot)
        self.volt_dock.addWidget(   self.volt.plot)
        self.gps_dock.addWidget(    self.gps.plot)
        self.airsp_dock.addWidget(  self.airsp.plot)
        self.part_dock.addWidget(   self.part.plot)
        
        self.window.show()
        self.app.exec_()
        """

    def plotWidgets(self, plots, units):
        # Creates a plot for each field and unit desired as defined in lists passed into function
        # Indicies of field and units are assumed to be equal
        # Placement needs to be worked on
        self.widget = QtGui.QWidget()
        self.layout = QtGui.QGridLayout()
        self.widget.setLayout(self.layout)
        for i,dock in enumerate(plots):
            self.dock_name = Dock(dock.title())
            if i == 0:
                self.area.addDock(self.dock_name)
            elif i % 2 > 0:
                self.area.addDock(self.dock_name, 'right', self.previous)
            elif i % 2 == 0:
                self.area.addDock(self.dock_name, 'below', self.previous)
            self.dock_plot = realplot.RTP(name = dock.title() + " ("+ units[i] + ")")
            self.dock_name.addWidget(self.dock_plot, i, 0, 1, 1)
            self.previous = self.dock_name

    def messageWidgets(self):
        # Creates the messages dock to contain all widgets that are not plots
        # Returns the name of this dock to be passed into other functions so that widgets may be directly added to it
        self.dock = Dock('Messsages')
        self.area.addDock(self.dock)
        return self.dock

    def buttonWidgets(self, buttons, dock):
        # Creates a button for each value in the list passed through function
        # Placement needs to be worked on
        self.widget = QtGui.QWidget() #will hold all other buttons in one widget
        self.layout = QtGui.QGridLayout() #layout for that widget
        dock.addWidget(self.widget)
        for i,button in enumerate(buttons):
            #print('error')
            self.button_name = QtGui.QPushButton(button.title())
            self.button_name.setCheckable(True)
            self.layout.addWidget(self.button_name, i, 2, 1, 2)
            #print('error2')
        self.widget.setLayout(self.layout)

    def execute(self):
        # Executes the application
        self.window.show()
        self.app.exec_()


    ## These functions will be worked on later
    #def serial(self, baud):
        # self.serial = serial.Serial()
        # self.serial.timeout = 0.02
        # self.serial.baudrate = 115200
        # self.serial.port = port
        # self.serial.open()

    #def graph(self, plot, data):
    #    plot.addpoint()

    #def parseSerial(self, serial):
    #    binary = serial.readline()
    #    data = str(binary, encoding='ascii')
    #    data.strip().split(',')


def testClass(title, names, units, buttons, width, height):
    # Test for the groundStation class, development purposes only
    gs = groundStation(title, width, height)
    messages = gs.messageWidgets()
    gs.plotWidgets(names, units)
    gs.buttonWidgets(buttons, messages)
    gs.execute()

if __name__ == "__main__":
    plots = ["altitude", "pressure", "voltage", "airspeed"]
    units = ["m", "Pa", "V", "m/s"]
    buttons = ["reset", "calibrate", "packet"]
    width = 1600
    height = 1200
    testClass("ragnarok", plots, units, buttons, width, height)