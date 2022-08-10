'''
Where the GUI application is created and executed from
'''

import logging
import logging.config
from logger_config import LOG_DICT_CONFIG

logging.config.dictConfig(LOG_DICT_CONFIG)

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QHBoxLayout
from PyQt5.QtCore import QTimer, Qt
from pyqtgraph import PlotWidget
from pyqtgraph.opengl import GLViewWidget, GLAxisItem, GLScatterPlotItem, GLGridItem
import sys
import numpy as np

from anchors import ANCHOR_X, ANCHOR_Y
from tag_serial_interface import TagInterface

logger = logging.getLogger(__name__)

class MainWindow(QWidget):
    def __init__(self, port):

        super().__init__()
        self.setWindowTitle("Localisation for HMS")

        self.plot_widget = PlotWidget()
        self.plot_widget.setMinimumSize(300, 300)
        self.plot_widget.setBackground('w')

        self.quality_label = QLabel('0')
        self.quality_slider = QSlider(Qt.Horizontal)
        self.quality_slider.setMinimum(0)
        self.quality_slider.setMaximum(100)
        self.quality_slider.setValue(0)

        quality_layout = QHBoxLayout()
        quality_layout.addWidget(QLabel('Quality:'))
        quality_layout.addWidget(self.quality_label)
        quality_layout.addWidget(self.quality_slider)

        v_layout = QVBoxLayout()
        v_layout.addWidget(self.plot_widget)
        v_layout.addLayout(quality_layout)
        
        self.layout = QHBoxLayout()
        self.layout.addLayout(v_layout)

        self.widget_3d = GLViewWidget()
        self.widget_3d.setFixedSize(400, 400)
        self.layout.addWidget(self.widget_3d)
        #self.widget_3d.setBackgroundColor('w')

        self.setLayout(self.layout)

        self.axis_3d = GLAxisItem()
        self.widget_3d.addItem(self.axis_3d)
        self.axis_3d.setSize(x=5,y=5,z=5)

        '''
        gx = GLGridItem()
        gx.rotate(90, 0, 1, 0)
        gx.translate(-10, 0, 0)
        self.widget_3d.addItem(gx)
        gy = GLGridItem()
        gy.rotate(90, 1, 0, 0)
        gy.translate(0, -10, 0)
        self.widget_3d.addItem(gy)
        gz = GLGridItem()
        gz.translate(0, 0, -10)
        self.widget_3d.addItem(gz)
        '''

        self.scatter_3d = GLScatterPlotItem()
        self.widget_3d.addItem(self.scatter_3d)

        self.anchor_plot = self.plot_widget.plot(ANCHOR_X, ANCHOR_Y, pen=None, symbol='o',
                symbolBrush='b')

        self.plot_widget.setXRange(min(ANCHOR_X), max(ANCHOR_X))
        self.plot_widget.setYRange(min(ANCHOR_Y), max(ANCHOR_Y))

        self.interface = TagInterface(port)
        self.interface.reset()
        fall, px, py, qf, ax, ay, az = self.interface.read_data()
        self.tag_plot = self.plot_widget.plot([px], [py], pen=None, symbol='o',
                symbolBrush='g')

        self.fall_plot = self.plot_widget.plot([], [], pen=None, symbol='x', symbolBrush='r')
        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update)
        self.timer.start()
        logger.info('App configured')

    def update(self):
        fall, px, py, qf, ax, ay, az = self.interface.read_data()
        self.tag_plot.setData([px], [py])
        self.quality_slider.setValue(qf)
        self.quality_label.setText(str(qf))

        if fall:
            logger.warning('Fall detected at position %d, %d', px, py)
            self.fall_plot.setData([px], [py])
        
        
        self.scatter_3d.setData(pos=np.array([ax,ay,az])/9000)
        logger.debug("ax: %d ay: %d az: %d", ax, ay, az)
        #logger.debug('Updating GUI') 
    
if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow(sys.argv[1])
    window.show()
    logger.info('Starting GUI')
    app.exec()

