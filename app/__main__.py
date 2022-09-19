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
from scipy.ndimage import median_filter
from functools import partial
from collections import deque
from statistics import fmean, StatisticsError
import enum

from anchors import ANCHOR_X, ANCHOR_Y
from tag_serial_interface import TagInterface

logger = logging.getLogger(__name__)
pos_logger = logging.getLogger('pos_logger')
accel_logger = logging.getLogger('accel_logger')

ACC_SAMPLE_RATE = 100 # Hz
LPF_CUTOFF_HIGH = 15 #Hz
LPF_ORDER = 12
GAIN_FACTOR = 500

L = 600 #window size for rolling calculations

RMS_THRESHOLD = 2.5
SNR_THRESHOLD = 16 #db

N_SAMPLES_FOR_MOVEMENT_DIRECTION = 150 #how many samples to use to infer movement direction

# Parameters for step length estimation
A = 0.868
B = -0.716

TAG_POLL_INTERVAL = 5 #1/ACC_SAMPLE_RATE * 1000 # ms
#analyse using overlapping samples for good FFT frequency resolution and decent (1Hz) fft update rate
MOTION_ANALYSIS_INTERVAL = 1/ACC_SAMPLE_RATE * 1000 * L/6
GRAPH_UPDATE_INTERVAL = 140 #ms

MEASURE_CONFIDENCE = 0.9

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

        self.prev_x = deque(maxlen=N_SAMPLES_FOR_MOVEMENT_DIRECTION)
        self.prev_y = deque(maxlen=N_SAMPLES_FOR_MOVEMENT_DIRECTION)

        self.movement_direction = None

        self.fft = self.fft_freq = None

        self.step_frequency = None

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
        if self.predicted_x is not None and self.movement_direction is not None and self.step_length is not None:
            pos_logger.info('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s', measured_x, measured_y, self.predicted_x, self.predicted_y, self.movement_direction, self.step_length, self.motion_status.name)
        else:
            pos_logger.info('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s', measured_x, measured_y, float('nan'), float('nan'), float('nan'), float('nan'), self.motion_status.name)

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
        if len(self.prev_x) > 0:
            # Take of inferred direction of all recorded previous samples and
            # average to reduce noise in direction estimation
            cumulative_movement_direction = 0
            for (past_x, past_y) in zip(self.prev_x, self.prev_y):
                delta_x = past_x - self.px
                delta_y = past_y - self.py
                tmp_movement_direction = np.degrees(np.arctan2(delta_y, delta_x))
                tmp_movement_direction = -np.sign(tmp_movement_direction)*(180 - abs(tmp_movement_direction))
                cumulative_movement_direction += tmp_movement_direction
            self.movement_direction = cumulative_movement_direction / len(self.prev_x)
        else:
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
        f_ax = self.LPFS['x']([ax])[0]*GAIN_FACTOR
        f_ay = self.LPFS['y']([ay])[0]*GAIN_FACTOR
        f_az = self.LPFS['z']([az])[0]*GAIN_FACTOR
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
        
        self.a_ac_values.append(a_mag_ac)

        self.prev_x.append(self.px)
        self.prev_y.append(self.py)

        mutex.unlock()

    def analyse_motion(self):
        mutex.lock()
        # Compute fft
        # Apply median filtering to remove noise spikes
        a_ac_arr = np.array(self.a_ac_values)
        #a_ac_arr = median_filter(a_ac_arr, size=3)

        self.fft = np.abs(np.fft.rfft(a_ac_arr) / a_ac_arr.size).real
        self.fft_freq = ACC_SAMPLE_RATE * np.fft.rfftfreq(a_ac_arr.shape[-1])

        self.step_frequency  = None
        self.step_length = None

        #Infer motion status
        # Stationary versus motion
        self.rms = np.sqrt(1/a_ac_arr.size * np.sum(a_ac_arr**2))
        
        if self.rms <= RMS_THRESHOLD:
            # stationary
            logger.info('RMS: %f. Motion stationary', self.rms)
            self.motion_status = MotionStatus.STATIONARY
            self.motion_analysed.emit()
            mutex.unlock()
            return
        
        # Categorise erratic or walking motion

        # extract peak magnitude between 0.7 and 3 Hz
        possible_walk_frequencies = np.ma.masked_outside(self.fft_freq, 0.7, 3)
        possible_walk_magnitudes = np.ma.masked_where(possible_walk_frequencies.mask, self.fft)

        # find maximum magnitude in this range (frequency at which this occurs is the walk frequency)
        max_index = np.argmax(possible_walk_magnitudes)
        peak_mag = possible_walk_magnitudes[max_index]

        # Compute SNR where signal is the peak magnitude and everything else
        # in spectrum is noise.
        # N.B. only consider within possible walking range of 0.7 to 3Hz
        signal_power = peak_mag**2
        # compute noise power as average of other signals 
        noise_power = (np.sum(self.fft**2) - signal_power)/(self.fft.size - 1)

        SNR = 10*np.log10(signal_power/noise_power)

        # erratic motion
        if SNR <= SNR_THRESHOLD:
            logger.info('RMS: %f. SNR: %f. Motion erratic', self.rms, SNR)
            self.motion_status = MotionStatus.ERRATIC
        else:
            logger.info('RMS: %f. SNR: %f. Motion walking', self.rms, SNR)
            self.motion_status = MotionStatus.WALKING
            
            self.step_frequency = possible_walk_frequencies[max_index]

            # Use linear motion model to compute step length
            self.step_length = (A*self.step_frequency + B)*1000 # step length in mm
            logger.info("Step frequency: %f. Step length: %f", self.step_frequency, self.step_length)

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

        #self.plot_widget.setXRange(min(ANCHOR_X), max(ANCHOR_X))
        #self.plot_widget.setYRange(min(ANCHOR_Y), max(ANCHOR_Y))
        self.plot_widget.setXRange(-4000, 10000)
        self.plot_widget.setYRange(0, 15000)

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
        self.acceleration_ac_graph.addLine(x=None, y=RMS_THRESHOLD, pen=mkPen('r', width=1))
        self.rms_line = self.acceleration_ac_graph.addLine(x=None, y=0, pen=mkPen('g', width=1))
        
        self.fft_graph = PlotWidget(title='FFT of acceleration magnitude')
        self.fft_graph.setBackground('w')

        accel_layout.addWidget(self.fft_graph)

        self.fft_plot = self.fft_graph.plot([], [])

        # setup timers and worker
        self.worker = Worker(port)
        self.thread = QThread()
        self.poll_timer = QTimer()
        self.motion_analysis_timer = QTimer()

        self.worker.moveToThread(self.thread)
        self.poll_timer.moveToThread(self.thread)
        self.motion_analysis_timer.moveToThread(self.thread)

        self.poll_timer.setInterval(TAG_POLL_INTERVAL)
        self.motion_analysis_timer.setInterval(MOTION_ANALYSIS_INTERVAL)

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
            self.rms_line.setValue(self.worker.rms)
            mask = self.worker.fft_freq < 15

            self.fft_plot.setData(self.worker.fft_freq[mask], self.worker.fft[mask])
            #self.fft_plot.setData(self.worker.fft_freq, self.worker.fft)

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

