import numpy as np
import sys

ACC_SAMPLE_RATE = 50 #Hz

f_ax, f_ay, f_az = [], [], []
with open(sys.argv[1], 'r') as accel_file:
    for line in accel_file:
        _, ax, ay, az = line.split(',')
        f_ax.append(ax)
        f_ay.append(ay)
        f_az.append(az)

f_ax = np.array(f_ax, dtype=np.float64)
f_ay = np.array(f_ay, dtype=np.float64)
f_az = np.array(f_az, dtype=np.float64)

mag = np.sqrt(f_ax**2 + f_ay**2 + f_az**2)
ac_mag = mag - np.mean(mag)

fft = np.abs(np.fft.rfft(ac_mag)/ ac_mag.size).real
fft_freq = ACC_SAMPLE_RATE * np.fft.rfftfreq(ac_mag.shape[-1])

with open('acceleration_fft_data_out.csv', 'w') as out_file:
    for fft_value, fft_frequency in zip(fft, fft_freq):
        out_file.write(f"{fft_frequency},{fft_value}\n")

