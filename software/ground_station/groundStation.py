import realplot
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph import console
from pyqtgraph.dockarea import *
import serial
import string

class groundStation:
    def __init__(self, title):#, graphs=None):
        #self.graphs = []
        self.app = QtGui.QApplication([])
        self.window = QtGui.QMainWindow()
        self.area = DockArea()
        self.window.setCentralWidget(self.area)
        self.window.resize(1600, 1200)
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

    def widgets(self, names, units):
        for i,dock in enumerate(names):
            self.dock_name = Dock(dock.title())
            self.area.addDock(self.dock_name)
            self.dock_plot = realplot.RTP(name = dock.title() + " ("+ units[i] + ")")
            #self.dock = str(names[dock]).lower()
            self.dock_name.addWidget(self.dock_plot, i, 1, 1, 1)

    def execute(self):
        self.window.show()
        self.app.exec_()

    #def serial(self, baud):
        # self.serial = serial.Serial()
        # self.serial.timeout = 0.02
        # self.serial.baudrate = 115200
        # self.serial.port = port
        # self.serial.open()

def testclass(title, names, units):
    #groundStation(title)
    gs = groundStation(title)
    gs.widgets(names, units)
    gs.execute()

if __name__ == "__main__":
    names = ["altitude", "pressure"]
    units = ["m", "Pa"]
    testclass("ragnarok", names, units)