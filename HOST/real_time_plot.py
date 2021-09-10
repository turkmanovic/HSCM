from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import sys
import socket

PORT = 5050
SERVER = "192.168.1.101"
ADDR = (SERVER, PORT)


server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(ADDR)
fileToWrite = open('CurrentMeasurement.txt', 'w')

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.numberofline = 1

        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)

        msgAndAddress = server.recvfrom(16)
        incName = msgAndAddress[0]
        idPacket = self.getValue(0, incName)
        currentHigh = self.getValue(4, incName)
        currentLow = self.getValue(8, incName)
        timeStamp = self.getValue(12, incName)

        current = currentLow/1000
        # print(current)

        fileToWrite.write('Current measurement : \n')
        fileToWrite.write(str(self.numberofline) + ', currHigh: ' + str(currentHigh) + ', currLow: ' + str(currentLow/1000) + ', timeStamp: ' + str(timeStamp) + '\n')




        styles = {'font-size':'20px', 'color':'b'}
        self.graphWidget.setTitle("Current consumption of uC", size="20pt", color="k")
        self.graphWidget.setYRange(0, 150)
        self.graphWidget.setLabel('left', 'Current [mA]', **styles)
        self.graphWidget.setLabel('bottom', "Sample", **styles)
        self.x = list(range(timeStamp-150,timeStamp)) # 150 time points
        self.y = [ 0 for _ in range(150)]  # 150 data points
        self.graphWidget.setBackground('w')


        pen = pg.mkPen(color=(255, 0, 0))
        self.data_line = self.graphWidget.plot(self.x, self.y, pen=pen)
        self.timer = QtCore.QTimer()
        self.timer.setInterval(1)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def getValue(self, index, incName): #Parsing received bytes

        value = incName[0 + index] + incName[1 + index]*0xFF + incName[2+index]*0xFF00 + incName[3 + index]*0xFF0000
        return value


    def update_plot_data(self):

        msgAndAddress = server.recvfrom(16)
        incName = msgAndAddress[0]
        idPacket = self.getValue(0, incName)
        currentHigh = self.getValue(4, incName)
        currentLow = self.getValue(8, incName)
        timeStamp = self.getValue(12, incName)

        current = currentLow/1000
        print(timeStamp)


        self.x = self.x[1:]  # Remove the first y element.
        self.x.append(timeStamp)  # Add a new value 1 higher than the last.

        self.y = self.y[1:]  # Remove the first
        self.y.append(current)  # Add a new random value.
        self.numberofline = self.numberofline + 1
        fileToWrite.write(str(self.numberofline) + ', currHigh: ' + str(currentHigh) + ', currLow: ' + str(currentLow/1000) + ', timeStamp: ' + str(timeStamp) + '\n')
        self.data_line.setData(self.x, self.y)  # Update the data.



app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())

