import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import random
import time

class RTP:
    def __init__(self, name, x_val=None, y_val=None):
        self.x_val = []
        self.y_val = []
        if x_val is not None: self.x_val = x_val
        if y_val is not None: self.y_val = y_val
        self.plot = pg.PlotWidget(title=name)
        self.plot.plot(self.x_val, self.y_val)

    def addpoint(self, x, y):
        self.x_val.append(x)
        self.y_val.append(y)
        self.plot.plot(self.x_val, self.y_val)

    def clear(self):
        self.plot.clear()
        self.x_val = []
        self.y_val = []

def testclass():
    app = QtGui.QApplication([])
    window = QtGui.QWidget()
    layout = QtGui.QGridLayout()
    window.setLayout(layout)
    plot = RTP(name="test")
    layout.addWidget(plot.plot, 0, 0)
    window.show()

    num_points = int(input("How many random points would you like to test?\n"))
    start_time = time.time()
    for elem in range(num_points):
        plot.addpoint(elem, random.randint(0, num_points))
        app.processEvents()
    end_time = time.time()
    runtime = end_time - start_time
    print("Runtime: %5.2f" %runtime)

    app.exec_()

if __name__ == "__main__":
    testclass()