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
import math
from scipy.signal import butter, lfilter
from functools import partial
from collections import deque
from statistics import mean, StatisticsError

from anchors import ANCHOR_X, ANCHOR_Y
from tag_serial_interface import TagInterface

logger = logging.getLogger(__name__)
ACC_SAMPLE_RATE = 50 # Hz
LPF_CUTOFF = 15 #Hz
NYQ_FREQ = 0.5*ACC_SAMPLE_RATE
LPF_ORDER = 10

L = 400 #window size for rolling calculations
TAG_POLL_INTERVAL = 20 # ms
GRAPH_UPDATE_INTERVAL = 100 #ms

class MainWindow(QWidget):
    def __init__(self, port):

        super().__init__()
        self.setWindowTitle("Localisation for HMS")

        self.plot_widget = PlotWidget()
        self.plot_widget.setMinimumSize(300, 300)
        self.plot_widget.setBackground('w')

        self.anchor_plot = self.plot_widget.plot(ANCHOR_X, ANCHOR_Y, pen=None, symbol='o',
                symbolBrush='b')
        self.fall_plot = self.plot_widget.plot([], [], pen=None, symbol='x', symbolBrush='r')
        self.tag_plot = self.plot_widget.plot([], [], pen=None, symbol='o',
                symbolBrush='g')

        self.plot_widget.setXRange(min(ANCHOR_X), max(ANCHOR_X))
        self.plot_widget.setYRange(min(ANCHOR_Y), max(ANCHOR_Y))

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
        self.setLayout(self.layout)

        '''
        self.widget_3d = GLViewWidget()
        self.widget_3d.setFixedSize(400, 400)
        self.layout.addWidget(self.widget_3d)
        #self.widget_3d.setBackgroundColor('w')

        self.setLayout(self.layout)
        self.axis_3d = GLAxisItem(glOptions='opaque')
        self.widget_3d.addItem(self.axis_3d)
        self.axis_3d.setSize(x=5,y=5,z=5)

        self.scatter_3d = GLScatterPlotItem()
        self.widget_3d.addItem(self.scatter_3d)
        '''

        accel_layout = QVBoxLayout()
        self.layout.addLayout(accel_layout)
        self.acceleration_graph = PlotWidget()
        accel_layout.addWidget(self.acceleration_graph)
        self.acceleration_graph.setBackground('w')
        self.acceleration_graph.setXRange(0, L)
        self.ax_plot = self.acceleration_graph.plot([], [], symbolBrush='r', symbolSize=4)
        self.ay_plot = self.acceleration_graph.plot([], [], symbolBrush='g', symbolSize=4)
        self.az_plot = self.acceleration_graph.plot([], [], symbolBrush='b', symbolSize=4)

        self.acceleration_ac_graph = PlotWidget()
        accel_layout.addWidget(self.acceleration_ac_graph)
        self.acceleration_ac_graph.setBackground('w')
        self.acceleration_ac_graph.setXRange(0, L)

        self.a_ac_plot = self.acceleration_ac_graph.plot([], [])
        self.a_ac_plot.setFftMode(False)

        self.ax_values = deque(maxlen=L)
        self.ay_values = deque(maxlen=L)
        self.az_values = deque(maxlen=L)

        self.a_ac_values = deque(maxlen=L)

        self.energy_graph = PlotWidget()
        self.energy_graph.setBackground('w')
        self.cv_graph = PlotWidget()
        self.cv_graph.setBackground('w')

        accel_layout.addWidget(self.energy_graph)
        accel_layout.addWidget(self.cv_graph)

        self.energy_graph.setXRange(0, L)
        self.cv_graph.setXRange(0, L)

        self.energy_plot = self.energy_graph.plot([], [])
        self.cv_plot = self.cv_graph.plot([], [])

        self.energy_values = deque(maxlen=L)
        self.cv_values = deque(maxlen=L)

        # set-up LPF
        bx, ax = butter(LPF_ORDER, LPF_CUTOFF/NYQ_FREQ, btype='low')
        by, ay = butter(LPF_ORDER, LPF_CUTOFF/NYQ_FREQ, btype='low')
        bz, az = butter(LPF_ORDER, LPF_CUTOFF/NYQ_FREQ, btype='low')

        self.LPFS = {'x': partial(lfilter, bx, ax), 'y': partial(lfilter, by, ay), 'z': partial(lfilter, bz, az)}

        # set up tag interface 
        self.interface = TagInterface(port)
        self.interface.reset()

        # setup update timer
        self.poll_timer = QTimer()
        self.poll_timer.setInterval(TAG_POLL_INTERVAL)
        self.poll_timer.timeout.connect(self.poll)

        self.update_timer = QTimer()
        self.update_timer.setInterval(GRAPH_UPDATE_INTERVAL)
        self.update_timer.timeout.connect(self.update_gui)


        self.px = self.py = self.qf = self.fall_px = self.fall_py = 0

        logger.info('App configured, starting..')
        self.poll_timer.start()
        self.update_timer.start()

    def update_gui(self):
        pass
        # Update tag position plot
        self.tag_plot.setData([self.px], [self.py])
        
        # update quality indicator
        self.quality_slider.setValue(self.qf)
        self.quality_label.setText(str(self.qf))

        # Update fall marker
        self.fall_plot.setData([self.fall_px], [self.fall_py])

        # update filtered acceleration plots: ax ay az
        self.ax_plot.setData(range(len(self.ax_values)), self.ax_values)
        self.ay_plot.setData(range(len(self.ay_values)), self.ay_values)
        self.az_plot.setData(range(len(self.az_values)), self.az_values)

        # Update acceleration AC magnitude plot
        self.a_ac_plot.setData(range(len(self.a_ac_values)), self.a_ac_values)

        # Update energy and cv plots
        self.energy_plot.setData(range(len(self.energy_values)), self.energy_values)
        self.cv_plot.setData(range(len(self.cv_values)), self.cv_values)

    def poll(self):
        fall, self.px, self.py, self.qf, ax, ay, az = self.interface.read_data()

        if fall:
            logger.warning('Fall detected at position %d, %d', self.px, self.py)
            self.fall_px, self.fall_py = self.px, self.py
        
        # Digital low pass filter values
        f_ax, f_ay, f_az = *self.LPFS['x']([ax]), *self.LPFS['y']([ay]), *self.LPFS['z']([az])
        self.ax_values.append(f_ax)
        self.ay_values.append(f_ay)
        self.az_values.append(f_az)

        # Take magnitude of acceleration data
        a_mag = math.sqrt(f_ax**2 + f_ay**2 + f_az**2)

        # Remove DC component
        try:
            mean_val = mean(self.a_ac_values)
            a_mag_ac = a_mag - mean_val
        except StatisticsError:
            #FIXME: is there a better way
            # if we do not have any previous samples, then assume
            # 0 AC value.
            a_mag_ac = 0
            mean_val = 0

        self.a_ac_values.append(a_mag_ac)

        # Compute and plot energy and cv
        a_ac_arr = np.array(self.a_ac_values)
        energy = np.sqrt(1/len(self.a_ac_values) * np.sum(a_ac_arr**2))
        cv = np.sqrt(1/len(self.a_ac_values) * np.sum((a_ac_arr - mean_val)**2))/mean_val
        #logger.debug('Energy: %f. cv: %f', energy, cv)

        self.energy_values.append(energy)
        self.cv_values.append(cv)

if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow(sys.argv[1])
    window.show()
    logger.info('Starting GUI')
    app.exec()

