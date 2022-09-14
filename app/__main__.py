'''
Where the GUI application is created and executed from
'''

import logging
import logging.config
from logger_config import LOG_DICT_CONFIG

logging.config.dictConfig(LOG_DICT_CONFIG)

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QHBoxLayout
from PyQt5.QtCore import QTimer, Qt, QObject, QMutex, QThread, pyqtSignal
from pyqtgraph import PlotWidget, mkPen, ArrowItem
import sys
import numpy as np
import math
from scipy.signal import butter, sosfilt
from functools import partial
from collections import deque
from statistics import fmean, StatisticsError
import enum

from anchors import ANCHOR_X, ANCHOR_Y
from tag_serial_interface import TagInterface

logger = logging.getLogger(__name__)
pos_logger = logging.getLogger('pos_logger')
accel_logger = logging.getLogger('accel_logger')

ACC_SAMPLE_RATE = 50 # Hz
LPF_CUTOFF_HIGH = 15 #Hz
LPF_CUTOFF_LOW = 0.1 #Hz
NYQ_FREQ = 0.5*ACC_SAMPLE_RATE
LPF_ORDER = 12

L = 100 #window size for rolling calculations

RMS_THRESHOLD = 4
FFT_THRESHOLD = 1
FFT_RELATIVE_PEAK_PERCENT = 20 # %

TAG_POLL_INTERVAL = 20 # ms
GRAPH_UPDATE_INTERVAL = 140 #ms

MEASURE_CONFIDENCE = 0.4

mutex = QMutex()

class MotionStatus(enum.Enum):
    STATIONARY = 0
    WALKING = 1
    ERRATIC = 2

MOTIONS_STATUS_COLOUR_MAP = {
    MotionStatus.STATIONARY: 'white',
    MotionStatus.WALKING: 'green',
    MotionStatus.ERRATIC: 'red',
}

class Worker(QObject):

    motion_analysed = pyqtSignal()

    def __init__(self, port):
        super().__init__()
        self.ax_values = deque(maxlen=L)
        self.ay_values = deque(maxlen=L)
        self.az_values = deque(maxlen=L)

        self.a_values = deque(maxlen=L) # before removing DC component - required for mean calculations.
        self.a_ac_values = deque(maxlen=L)

        self.rms_values = deque(maxlen=L)
        self.rms_mean = 0

        # set-up LPF
        sosx = butter(LPF_ORDER, LPF_CUTOFF_HIGH, btype='low', fs=ACC_SAMPLE_RATE, output='sos')
        sosy = butter(LPF_ORDER, LPF_CUTOFF_HIGH, btype='low', fs=ACC_SAMPLE_RATE, output='sos')
        sosz = butter(LPF_ORDER, LPF_CUTOFF_HIGH, btype='low', fs=ACC_SAMPLE_RATE, output='sos')

        self.LPFS = {'x': partial(sosfilt, sosx), 'y': partial(sosfilt, sosy), 'z': partial(sosfilt, sosz)}

        # set up tag interface 
        self.interface = TagInterface(port)
        self.interface.reset()

        self.px = self.py = self.qf = 0
        self.fall_px = self.fall_py = None

        self.motion_status = MotionStatus.STATIONARY

        self.rolling_x = []
        self.rolling_y = []

        self.prev_x = deque(maxlen=5)
        self.prev_y = deque(maxlen=5)

        self.movement_direction = None

        self.fft = self.fft_freq = None

        self.fft_peak_frequency = None

        self.a = 0.01
        self.b = 0
        self.L = 1.00*1000 # metre in mm

        self.step_length = None
        self.predicted_x = self.predicted_y = None


    def poll(self):
        mutex.lock()

        try:
            fall, measured_x, measured_y, self.qf, ax, ay, az = self.interface.read_data()
        except ValueError:
            mutex.unlock()
            return

        if self.motion_status == MotionStatus.WALKING and self.predicted_x is not None:
            self.px = MEASURE_CONFIDENCE*measured_x + (1-MEASURE_CONFIDENCE)*self.predicted_x
            self.py = MEASURE_CONFIDENCE*measured_y + (1-MEASURE_CONFIDENCE)*self.predicted_y
        else:
            self.px = measured_x
            self.py = measured_y
        if self.predicted_x is not None:
            pos_logger.info('%.2f,%.2f,%.2f,%.2f,%s', measured_x, measured_y, self.predicted_x, self.predicted_y, self.motion_status.name)
        else:
            pos_logger.info('%.2f,%.2f,%.2f,%.2f,%s', measured_x, measured_y, float('nan'), float('nan'), self.motion_status.name)

        # if stationary we take rolling average so store pos values
        if self.motion_status == MotionStatus.STATIONARY:
            self.rolling_x.append(self.px)
            self.rolling_y.append(self.py)
        else:
            # otherwise clear the values
            self.rolling_x = []
            self.rolling_y = []

        if fall:
            logger.warning('Fall detected at position %d, %d', self.px, self.py)
            self.fall_px, self.fall_py = self.px, self.py
        
        # infer direction of motion from subsequent samples
        try:
            delta_x = self.prev_x[0] - self.px
            delta_y = self.prev_y[0] - self.py
            self.movement_direction = np.degrees(np.arctan2(delta_y, delta_x))
            self.movement_direction = -np.sign(self.movement_direction)*(180 - abs(self.movement_direction))
        except:
            # if we don't have previous position sample
            # then we cannot infer angle
            logger.debug("Missing previous samples, no movement direction set")
            self.movement_direction = None
        # then infer next position using estimated step length and direction
        if self.step_length is not None and self.movement_direction is not None:
            self.predicted_x = self.prev_x[-1] + np.cos(np.radians(self.movement_direction)) * self.step_length
            self.predicted_y = self.prev_y[-1] + np.sin(np.radians(self.movement_direction)) * self.step_length
        else:
            self.predicted_x = self.predicted_y = None

        # Digital low pass filter values
        f_ax = self.LPFS['x']([ax])[0]
        f_ay = self.LPFS['y']([ay])[0]
        f_az = self.LPFS['z']([az])[0]
        self.ax_values.append(f_ax)
        self.ay_values.append(f_ay)
        self.az_values.append(f_az)
        accel_logger.info('%.2f,%.2f,%.2f',f_ax,f_ay,f_az)

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
        
        # compute RMS
        self.a_ac_values.append(a_mag_ac)

        a_ac_arr = np.array(self.a_ac_values)

        rms = np.sqrt(1/len(self.a_ac_values) * np.sum(a_ac_arr**2))
        self.rms_values.append(rms)

        self.prev_x.append(self.px)
        self.prev_y.append(self.py)

        mutex.unlock()

    def analyse_motion(self):
        mutex.lock()
        # Compute fft
        a_ac_arr = np.array(self.a_ac_values)
        a_ac_mean_val = np.mean(a_ac_arr)

        self.fft = np.abs(np.fft.rfft(a_ac_arr) / a_ac_arr.size)
        self.fft_freq = ACC_SAMPLE_RATE * np.fft.rfftfreq(a_ac_arr.shape[-1])

        #Infer motion status
        self.rms_mean = np.mean(self.rms_values)
        
        # store initial peak frequency
        self.fft_peak_frequency = self.fft_freq[np.argmax(self.fft.real)]

        if self.rms_mean > RMS_THRESHOLD:
            # Take fft data. Find top 2 peaks. 
            idxs = np.argpartition(self.fft.real, -2)[-2:]
            sorted_idx = idxs[np.argsort(self.fft.real[idxs])][::-1]
            # if top peak below abs threshold, but we had sufficient
            # RMS value, consider motion erratic
            if self.fft.real[sorted_idx[0]] < FFT_THRESHOLD:
                self.motion_status = MotionStatus.ERRATIC
                logger.info('RMS exceeded threshold but FFT peak did not. Motion deemed erratic')
            else:
                # if top peak isn't FFT_RELATIVE_PEAK_PERCENT larger than 2nd, consider
                # motion erratic as no clear frequency
                top_mag = self.fft.real[sorted_idx[0]] 
                second_mag = self.fft.real[sorted_idx[1]]
                print(top_mag, second_mag)
                diff = top_mag - second_mag #always positive because we abs fft values
                if diff > (top_mag/100 * FFT_RELATIVE_PEAK_PERCENT):
                    logger.info('Walking motion detected')
                    self.motion_status = MotionStatus.WALKING
                    # save peak for step length estimation
                    self.fft_peak_frequency = self.fft_freq[sorted_idx[0]]
                    logger.debug("FFT peak frequency: %f", self.fft_peak_frequency)
                else:
                    logger.info('Relative difference of FFT peaks insufficent. Expected %f, got %f', top_mag/100*FFT_RELATIVE_PEAK_PERCENT, diff)
                    self.motion_status = MotionStatus.ERRATIC
        else:
            # Take fft data. Find top 2 peaks. 
            idxs = np.argpartition(self.fft.real, -2)[-2:]
            sorted_idx = idxs[np.argsort(self.fft.real[idxs])]
            # if top peak above abs threshold, but we had insufficient
            # RMS value, consider motion erratic
            if self.fft.real[sorted_idx[0]] > FFT_THRESHOLD:
                self.motion_status = MotionStatus.ERRATIC
                logger.info('FFT exceeded threshold but RMS peak did not. Motion deemed erratic')
            else:
                logger.info('Motion deemed stationary')
                self.motion_status = MotionStatus.STATIONARY
        
        # Use linear motion model to compute step length
        self.step_length = self.a * self.L * self.fft_peak_frequency + self.b
        logger.info("Predicted step length: %f", self.step_length)

        self.motion_analysed.emit()
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

        self.rolling_tag_plot = self.plot_widget.plot([], [], pen=None, symbol='o',
                symbolBrush='k')
        
        self.predicted_position_plot = self.plot_widget.plot([], [], pen=None, symbol='o', symbolBrush='r')
        self.movement_direction_arrow = ArrowItem(tailLen=20, brush='y')
        self.plot_widget.addItem(self.movement_direction_arrow)
        # initially hidden
        self.movement_direction_arrow.setVisible(False)

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

        self.motion_status = QLabel('Stationary')
        self.motion_status.setStyleSheet('background-color:white')
        self.motion_status.setAlignment(Qt.AlignCenter)

        v_layout = QVBoxLayout()
        v_layout.addWidget(self.plot_widget)
        v_layout.addLayout(quality_layout)
        v_layout.addWidget(self.motion_status)
        
        self.layout = QHBoxLayout()
        self.layout.addLayout(v_layout)
        self.setLayout(self.layout)

        accel_layout = QVBoxLayout()
        self.layout.addLayout(accel_layout)
        self.acceleration_graph = PlotWidget(title='Filtered acceleration data')
        accel_layout.addWidget(self.acceleration_graph)
        self.acceleration_graph.setBackground('w')
        self.acceleration_graph.setXRange(0, L)
        self.ax_plot = self.acceleration_graph.plot([], [], symbolBrush='r', symbolSize=4)
        self.ay_plot = self.acceleration_graph.plot([], [], symbolBrush='g', symbolSize=4)
        self.az_plot = self.acceleration_graph.plot([], [], symbolBrush='b', symbolSize=4)

        self.acceleration_ac_graph = PlotWidget(title='Acceleration magnitude (DC component removed)')
        accel_layout.addWidget(self.acceleration_ac_graph)
        self.acceleration_ac_graph.setBackground('w')
        self.acceleration_ac_graph.setXRange(0, L)

        self.a_ac_plot = self.acceleration_ac_graph.plot([], [])

        self.rms_graph = PlotWidget(title='Acceleration magnitude RMS')
        self.rms_graph.setBackground('w')
        self.fft_graph = PlotWidget(title='FFT of acceleration magnitude')
        self.fft_graph.setBackground('w')

        accel_layout.addWidget(self.rms_graph)
        accel_layout.addWidget(self.fft_graph)

        self.rms_graph.setXRange(0, L)

        self.rms_plot = self.rms_graph.plot([], [])
        self.rms_graph.addLine(x=None, y=RMS_THRESHOLD, pen=mkPen('r', width=1))
        self.rms_mean_line = self.rms_graph.addLine(x=None, y=0, pen=mkPen('g', width=1))
        self.fft_plot = self.fft_graph.plot([], [])
        self.fft_graph.addLine(x=None, y=FFT_THRESHOLD, pen=mkPen('r', width=1))

        # setup timers and worker
        self.worker = Worker(port)
        self.thread = QThread()
        self.poll_timer = QTimer()
        self.motion_analysis_timer = QTimer()

        self.worker.moveToThread(self.thread)
        self.poll_timer.moveToThread(self.thread)
        self.motion_analysis_timer.moveToThread(self.thread)

        self.poll_timer.setInterval(TAG_POLL_INTERVAL)
        # only analyse motion every window interval
        self.motion_analysis_timer.setInterval(L*(1000/ACC_SAMPLE_RATE))

        self.poll_timer.timeout.connect(self.worker.poll)
        self.motion_analysis_timer.timeout.connect(self.worker.analyse_motion)

        self.thread.started.connect(self.poll_timer.start)
        self.thread.started.connect(self.motion_analysis_timer.start)

        self.thread.finished.connect(self.poll_timer.stop)
        self.thread.finished.connect(self.motion_analysis_timer.stop)
        self.thread.finished.connect(self.worker.shutdown)

        self.update_timer = QTimer()
        self.update_timer.setInterval(GRAPH_UPDATE_INTERVAL)
        self.update_timer.timeout.connect(self.update_gui)


        # when motion analysed, update GUI
        self.worker.motion_analysed.connect(self.update_motion)

        logger.info('App configured, starting..')
        self.thread.start()
        self.update_timer.start()

    def closeEvent(self, event):
        self.thread.exit()
        self.thread.wait()

        event.accept() # close

    def update_gui(self):
        mutex.lock()

        # Update tag position plot
        self.tag_plot.setData([self.worker.px], [self.worker.py])
        if self.worker.motion_status == MotionStatus.STATIONARY:
            self.rolling_tag_plot.setData([np.mean(self.worker.rolling_x)], 
                [np.mean(self.worker.rolling_y)])
        else:
            self.rolling_tag_plot.setData([], [])

        if self.worker.motion_status == MotionStatus.WALKING:
            # add 180 to account for offset (0 degrees points left)
            self.movement_direction_arrow.setStyle(angle=180-self.worker.movement_direction)
            self.movement_direction_arrow.setPos(self.worker.px, self.worker.py)
            self.movement_direction_arrow.setVisible(True)
        else:
            self.movement_direction_arrow.setVisible(False)
        
        # show predicted position for sample
        if self.worker.predicted_x is not None:
            self.predicted_position_plot.setData([self.worker.predicted_x], [self.worker.predicted_y])
        else:
            self.predicted_position_plot.setData([], [])

        # update quality indicator
        self.quality_slider.setValue(self.worker.qf)
        self.quality_label.setText(str(self.worker.qf))

        # Update fall marker
        if self.worker.fall_px is not None:
            self.fall_plot.setData([self.worker.fall_px], [self.worker.fall_py])

        # update filtered acceleration plots: ax ay az
        self.ax_plot.setData(range(len(self.worker.ax_values)), self.worker.ax_values)
        self.ay_plot.setData(range(len(self.worker.ay_values)), self.worker.ay_values)
        self.az_plot.setData(range(len(self.worker.az_values)), self.worker.az_values)

        # Update acceleration AC magnitude plot
        self.a_ac_plot.setData(range(len(self.worker.a_ac_values)), self.worker.a_ac_values)

        mutex.unlock()

    def update_motion(self):
        mutex.lock()

        # Update rms and fft plots
        if self.worker.fft is not None:
            self.rms_plot.setData(range(len(self.worker.rms_values)), self.worker.rms_values)
            self.rms_mean_line.setValue(self.worker.rms_mean)
            self.fft_plot.setData(self.worker.fft_freq, self.worker.fft.real)

            # Update motion status

            self.motion_status.setText(self.worker.motion_status.name.title())
            self.motion_status.setStyleSheet(f'background-color:{MOTIONS_STATUS_COLOUR_MAP[self.worker.motion_status]}')

        mutex.unlock()

if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow(sys.argv[1])
    window.show()
    logger.info('Starting GUI')
    app.exec()

