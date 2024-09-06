#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: magsens_calib.py

Description: A factory magneto sensor procedure for 24 cubic oriented measurments of the earth magnetic field.

Author: Helmut Rohs (helmut.rohs@gmail.com)
Created: 2024-09-01

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import os
import serial
import numpy as np
from scipy.optimize import least_squares
import re
import math
import csv
from ahrs.common import Quaternion


def extract_compass_value(log_line):
    """
    Filters a log line with 'Compass.cpp' and extracts the three sensor raw values.
    Args:
    - log_line: one log line (string).
    Returns:
    - A tupel with three extracted sensor values.
    """
    # RE-Pattern of log line
    pattern = r"Compass\.cpp: raw x (-?\d+), y (-?\d+), z (-?\d+)"

    extracted_value = None

    # Search through log line
    match = re.search(pattern, log_line)
    if match:
        # Extract values
        x, y, z = map(int, match.groups())
        return x, y, z
    else:
        return None


def segment_vectors(ser, jitter_threshold, max_segment_size=100):
    """
    Read a sequence of measurements from serial port, based on a jitter-threshold and a target segment size.
    Der Jitter wird als Abweichung vom Mittelwert der bisher gesammelten Samples verstanden.
    Args:
    - ser: Open serial port.
    - jitter_threshold: Max. allowed jitter on segment (deviation from average).
    - max_segment_size: Target samples per segment.
    Returns:
    - Average of the segment.
    """

    data_segment = []  # Current segment
    start = True
    complete = False

    while not complete:
        data = []
        if ser.in_waiting > 0:  # check if data available on serial port
            try:
                # Read a line from the serial port
                data = ser.readline().decode('utf-8').rstrip()  # and decode
                #print(f"Received data: {data}")
            except Exception as e:
                print(f"Error: {e}")
        else:
            continue

        value = extract_compass_value(data)
        if value != None:
            if start:  # First vector, start a new segment
                data_segment.append(value)
                start = False
                continue

            # Average of the current segment
            segment_mean = np.mean(data_segment, axis=0)

            # Jitter of the current segment
            jitter = np.linalg.norm(np.array(value) - segment_mean)

            # Check if jitter exceeds threshold
            if jitter <= jitter_threshold:
                data_segment.append(value)
            else:
                # Restart segment
                print("Restart")
                data_segment = [value]

            if len(data_segment) >= max_segment_size:
                complete = True

    return np.mean(data_segment, axis=0)

def grp4idx_of_sample(sample, reference):
    """
    Helper routine to find the proper symmetric group S4 (octahedron) rotation for
    the measurement on a cube.
    """
    global grp4
    # Initialize minimum difference and index
    min_difference_norm = float('inf')
    min_index = -1
    # Apply each rotation to the sample vector and compute difference norms
    for i, quat in enumerate(grp4):
        rotated_vector = quat.rotate(sample)
        difference_norm = np.linalg.norm(rotated_vector - reference)
        #print('--', i, difference_norm, rotated_vector)
        # Check if this is the smallest difference norm found so far
        if difference_norm < min_difference_norm:
            min_difference_norm = difference_norm
            min_index = i

    return min_index

def convert_to_vector(row):
    # Helper to remove extra whitespace and convert each element to float, then create a NumPy vector
    return [float(value.strip()) for value in row]

"""
Main magnetometer factory calibration procedure.
 - read 24 oversampled sensor values
 - map the proper octahedron rotation to each measurement
 - Minimize bias and scale from acquired data
 - generate a report
"""
all_samples = []
try:
    port = '/dev/ttyUSB0'
    baudrate = 115200

    # Attempt to open a serial port
    ser_io = serial.Serial(port, baudrate, timeout=10)
    print("Serial port opened successfully!")

    print("Ready to start sampling data? (press a key ...)")
    input()

    # Collect 24 measurements
    i = 0
    while i < 24:
        print("Measurement ", i+1)
        ser_io.reset_input_buffer()
        avg_data_sample = segment_vectors(ser_io, 450)
        print(avg_data_sample)
        all_samples.append(avg_data_sample)
        i = i+1
        print("Press any key to procede...")
        input()

    # Close serial port
    ser_io.close()
    print(all_samples)

except Exception as e:
    print(f"Error: Could not open serial port: {e}")
    # Read samples from csv file
    script_directory = os.path.dirname(os.path.abspath(__file__))
    file = os.path.join(script_directory, 'magsens_i2c_samples.csv')
    with open(file, mode='r', newline='') as file:
        # Create a CSV-Reader
        csv_reader = csv.reader(file)
        # Process line by line
        for row in csv_reader:
            all_samples.append(convert_to_vector(row))



# Helper function to create a quaternion from axis and angle
def quat_from_axis(axis, angle):
    if axis == 'x':
        return Quaternion(rpy=[angle, 0, 0])
    elif axis == 'y':
        return Quaternion(rpy=[0, angle, 0])
    elif axis == 'z':
        return Quaternion(rpy=[0, 0, angle])
    else:
        raise ValueError("Invalid axis")

# Initialize a list to store the 24 quaternions
grp4 = [None] * 24

# Define the 24 quaternions based on rotations
# Increments per face
rot1 = quat_from_axis('z', math.radians(90))
rot2 = quat_from_axis('z', math.radians(180))
rot3 = quat_from_axis('z', math.radians(-90))

# Rotations around the Z-axis
grp4[0] = Quaternion()  # Identity quaternion
grp4[1] = rot1
grp4[2] = rot2
grp4[3] = rot3

# 90-degree rotation around the X-axis
rot = quat_from_axis('x', math.radians(90))
grp4[4] = rot
grp4[5] = Quaternion(rot1 @ rot)
grp4[6] = Quaternion(rot2 @ rot)
grp4[7] = Quaternion(rot3 @ rot)

# 180-degree rotation around the X-axis
rot = quat_from_axis('x', math.radians(180))
grp4[8] = rot
grp4[9] = Quaternion(rot1 @ rot)
grp4[10] = Quaternion(rot2 @ rot)
grp4[11] = Quaternion(rot3 @ rot)

# -90-degree rotation around the X-axis
rot = quat_from_axis('x', math.radians(-90))
grp4[12] = rot
grp4[13] = Quaternion(rot1 @ rot)
grp4[14] = Quaternion(rot2 @ rot)
grp4[15] = Quaternion(rot3 @ rot)

# Further rotations
rot = quat_from_axis('y', math.radians(90))
grp4[16] = rot
grp4[17] = Quaternion(rot1 @ rot)
grp4[18] = Quaternion(rot2 @ rot)
grp4[19] = Quaternion(rot3 @ rot)

# 90-degree rotation around the Y-axis
rot = quat_from_axis('y', math.radians(-90))
grp4[20] = rot
grp4[21] = Quaternion(rot1 @ rot)
grp4[22] = Quaternion(rot2 @ rot)
grp4[23] = Quaternion(rot3 @ rot)

# Print the quaternions
#for i, quat in enumerate(grp4):
#    print(f"Quaternion {i+1}: {quat} (type {type(quat)})")

# Define the ground truth vectors with the proper group4 rotation
rots = [None] * len(all_samples)
rotated_before = [None] * len(all_samples)

for i, sample in enumerate(all_samples):
    g4idx = grp4idx_of_sample(sample, all_samples[0])
    if g4idx < 0:
        print("Filed to find orientation for sample ", i)
        exit()
    rots[i] = grp4[g4idx]
    rotated_before[i] = rots[i].rotate(sample)


# Define the cost function to minimize
def cost_function(params, measured):
    global rots
    # Extract bias and scale parameters
    bias = params[0:3] * 1000
    scale = params[3:6]

    # Apply bias and scale calibration
    calibrated = (measured - bias) * scale

    # Rotate
    rotated = [None] * len(measured)
    for i, sample in enumerate(calibrated):
        rotated[i] = rots[i].rotate(sample)

    # Normalize calibrated vectors
    rotated = rotated / np.linalg.norm(rotated, axis=1, keepdims=True)

    # Calculate the variance as residuals
    residual = np.var(rotated, axis=0)
    #print(bias, scale, residual)
    return residual[0] + residual[1] + residual[2] + (scale[0]-1)**2 + (scale[1]-1)**2 + (scale[2]-1)**2

# Initial guess for bias (zero) and scale (1 for each axis)
initial_guess = np.array([0, 0, 0, 1, 1, 1])

# Solve the least squares problem
result = least_squares(cost_function, initial_guess, args=([all_samples]))

# Extract calibrated parameters
calibrated_bias = result.x[0:3] * 1000
calibrated_scale = result.x[3:6]


print("Alle Rohdaten vom Magnetsensor und Distanz (vorher/nacher) zum ersten Messwert")

average_before = np.mean(rotated_before, axis=0)
results = (all_samples - calibrated_bias) * calibrated_scale
rotated = [None] * len(all_samples)
for i, sample in enumerate(results):
    rotated[i] = rots[i].rotate(sample)
average = np.mean(rotated, axis=0)
for i, sample in enumerate(results):
    difference_before = np.linalg.norm(rotated_before[i] - average_before)
    difference_norm = np.linalg.norm(rotated[i] - average)
    #print(i, rotated[i])
    print('\t %2d:\t (%.1f, %.1f, %.1f) - %5.0f /%5.0f' % (i+1, rotated_before[i][0], rotated_before[i][1], rotated_before[i][2], difference_before,  difference_norm))


# MagSens is shipping 1/15000 Gauss = (1/15000)/10000 T = 0,006667µT
uTesla = (1/15000)/10000*1000000 # x Gauss
print("STD vorher :", np.std(rotated_before, axis=0)*uTesla, "µT")

print("------------------------------------------")

print(f"Bias: {calibrated_bias}")
print(f"Scale: {calibrated_scale}")
print("RMS: %.2f µT" % (np.sum(np.linalg.norm(rotated - average, axis=1)) / len(all_samples) * uTesla))
print("STD nachher:", np.std(rotated, axis=0)*uTesla, "µT")

print("Mittlere Feldstärke: %.2f µT" % (np.sum(np.linalg.norm(rotated, axis=1))/len(all_samples)*uTesla))
