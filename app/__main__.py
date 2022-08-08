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
import sys

from anchors import ANCHOR_X, ANCHOR_Y
from tag_serial_interface import TagInterface

logger = logging.getLogger(__name__)

class MainWindow(QWidget):
    def __init__(self, port):

        super().__init__()
        self.setWindowTitle("Localisation for HMS")

        self.plot_widget = PlotWidget()
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

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.layout.addWidget(self.plot_widget)
        self.layout.addLayout(quality_layout)


        self.anchor_plot = self.plot_widget.plot(ANCHOR_X, ANCHOR_Y, pen=None, symbol='o',
                symbolBrush='r')

        self.plot_widget.setXRange(min(ANCHOR_X), max(ANCHOR_X))
        self.plot_widget.setYRange(min(ANCHOR_Y), max(ANCHOR_Y))

        self.interface = TagInterface(port)
        self.interface.reset()
        fall, px, py, qf, ax, ay, az = self.interface.read_data()
        self.tag_plot = self.plot_widget.plot([px], [py], pen=None, symbol='o',
                symbolBrush='g')
        self.timer = QTimer()
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.update_position)
        self.timer.start()
        logger.info('App configured')

    def update_position(self):
        fall, px, py, qf, ax, ay, az = self.interface.read_data()
        self.tag_plot.setData([px], [py])
        #self.quality_slider.setValue(qf)
        #self.quality_label.setText(str(qf))
        #logger.debug('Updating position') 
    
if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow(sys.argv[1])
    window.show()
    logger.info('Starting GUI')
    app.exec()

