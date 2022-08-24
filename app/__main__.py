'''
Where the GUI application is created and executed from
'''

import logging
import logging.config
from logger_config import LOG_DICT_CONFIG

logging.config.dictConfig(LOG_DICT_CONFIG)

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QHBoxLayout
from PyQt5.QtCore import QTimer, Qt, QObject, QMutex, QThread
from pyqtgraph import PlotWidget
from pyqtgraph.opengl import GLViewWidget, GLAxisItem, GLScatterPlotItem, GLGridItem
import sys
import numpy as np
import math
from scipy.signal import butter, sosfilt
from functools import partial
from collections import deque
from statistics import fmean, StatisticsError

from anchors import ANCHOR_X, ANCHOR_Y
from tag_serial_interface import TagInterface

logger = logging.getLogger(__name__)
ACC_SAMPLE_RATE = 50 # Hz
LPF_CUTOFF_HIGH = 15 #Hz
LPF_CUTOFF_LOW = 0.1 #Hz
NYQ_FREQ = 0.5*ACC_SAMPLE_RATE
LPF_ORDER = 12

L = 400 #window size for rolling calculations
TAG_POLL_INTERVAL = 20 # ms
GRAPH_UPDATE_INTERVAL = 20 #ms

mutex = QMutex()

class Worker(QObject):

    def __init__(self, port):
        super().__init__()
        self.ax_values = deque(maxlen=L)
        self.ay_values = deque(maxlen=L)
        self.az_values = deque(maxlen=L)

        self.a_values = deque(maxlen=L) # before removing DC component - required for mean calculations.
        self.a_ac_values = deque(maxlen=L)

        self.rms_values = deque(maxlen=L)
        self.std_dev_values = deque(maxlen=L)

        # set-up LPF
        sosx = butter(LPF_ORDER, LPF_CUTOFF_HIGH, btype='low', fs=ACC_SAMPLE_RATE, output='sos')
        sosy = butter(LPF_ORDER, LPF_CUTOFF_HIGH, btype='low', fs=ACC_SAMPLE_RATE, output='sos')
        sosz = butter(LPF_ORDER, LPF_CUTOFF_HIGH, btype='low', fs=ACC_SAMPLE_RATE, output='sos')

        self.LPFS = {'x': partial(sosfilt, sosx), 'y': partial(sosfilt, sosy), 'z': partial(sosfilt, sosz)}

        # set up tag interface 
        self.interface = TagInterface(port)
        self.interface.reset()

        self.px = self.py = self.qf = self.fall_px = self.fall_py = 0

    def poll(self):
        mutex.lock()

        try:
            fall, px, self.py, self.qf, ax, ay, az = self.interface.read_data()
        except ValueError:
            mutex.unlock()
            return

        if fall:
            logger.warning('Fall detected at position %d, %d', px, self.py)
            fall_px, self.fall_py = self.px, self.py
        
        # Digital low pass filter values
        f_ax = self.LPFS['x']([ax])[0]
        f_ay = self.LPFS['y']([ay])[0]
        f_az = self.LPFS['z']([az])[0]
        self.ax_values.append(f_ax)
        self.ay_values.append(f_ay)
        self.az_values.append(f_az)

        # Take magnitude of acceleration data
        a_mag = math.sqrt(f_ax**2 + f_ay**2 + f_az**2)

        # Remove DC component
        self.a_values.append(a_mag)
        try:
            mean_val = fmean(self.a_values)
            a_mag_ac = a_mag - mean_val
        except StatisticsError:
            # if we do not have any previous samples, then assume
            # 0 AC value.
            a_mag_ac = 0
            mean_val = 0
        
        self.a_ac_values.append(a_mag_ac)

        # Compute and plot rms and std_dev
        a_ac_arr = np.array(self.a_ac_values)
        a_ac_mean_val = np.mean(a_ac_arr)

        rms = np.sqrt(1/len(self.a_ac_values) * np.sum(a_ac_arr**2))
        std_dev = np.sqrt(1/len(self.a_ac_values) * np.sum((a_ac_arr - a_ac_mean_val)**2))

        self.rms_values.append(rms)
        self.std_dev_values.append(std_dev)

        mutex.unlock()

    def shutdown(self):
        self.interface.shutdown()
        
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

        self.rms_graph = PlotWidget()
        self.rms_graph.setBackground('w')
        self.std_dev_graph = PlotWidget()
        self.std_dev_graph.setBackground('w')

        accel_layout.addWidget(self.rms_graph)
        accel_layout.addWidget(self.std_dev_graph)

        self.rms_graph.setXRange(0, L)
        self.std_dev_graph.setXRange(0, L)

        self.rms_plot = self.rms_graph.plot([], [])
        self.std_dev_plot = self.std_dev_graph.plot([], [])

        # setup timers and worker
        self.worker = Worker(port)
        self.thread = QThread()
        self.poll_timer = QTimer()

        self.worker.moveToThread(self.thread)
        self.poll_timer.moveToThread(self.thread)

        self.poll_timer.setInterval(TAG_POLL_INTERVAL)
        self.poll_timer.timeout.connect(self.worker.poll)
        self.thread.started.connect(self.poll_timer.start)
        self.thread.started.connect(self.poll_timer.start)
        self.thread.finished.connect(self.poll_timer.stop)
        self.thread.finished.connect(self.worker.shutdown)

        self.update_timer = QTimer()
        self.update_timer.setInterval(GRAPH_UPDATE_INTERVAL)
        self.update_timer.timeout.connect(self.update_gui)

        logger.info('App configured, starting..')
        self.thread.start()
        self.update_timer.start()

    def closeEvent(self, event):
        self.thread.exit()

        event.accept() # close

    def update_gui(self):
        mutex.lock()

        # Update tag position plot
        self.tag_plot.setData([self.worker.px], [self.worker.py])
        
        # update quality indicator
        self.quality_slider.setValue(self.worker.qf)
        self.quality_label.setText(str(self.worker.qf))

        # Update fall marker
        self.fall_plot.setData([self.worker.fall_px], [self.worker.fall_py])

        # update filtered acceleration plots: ax ay az
        self.ax_plot.setData(range(len(self.worker.ax_values)), self.worker.ax_values)
        self.ay_plot.setData(range(len(self.worker.ay_values)), self.worker.ay_values)
        self.az_plot.setData(range(len(self.worker.az_values)), self.worker.az_values)

        # Update acceleration AC magnitude plot
        self.a_ac_plot.setData(range(len(self.worker.a_ac_values)), self.worker.a_ac_values)

        # Update rms and std_dev plots
        self.rms_plot.setData(range(len(self.worker.rms_values)), self.worker.rms_values)
        self.std_dev_plot.setData(range(len(self.worker.std_dev_values)), self.worker.std_dev_values)

        mutex.unlock()

if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow(sys.argv[1])
    window.show()
    logger.info('Starting GUI')
    app.exec()

