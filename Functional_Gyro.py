import time
import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

##########################
# Configuration
##########################
PORT = "COM7"  # Replace with your Arduino's COM port
BAUD_RATE = 115200

# Adjust these for smoothing or integration
UPDATE_RATE = 50.0   # Hz (times per second we update)
DT = 1.0 / UPDATE_RATE
SMOOTHING_ALPHA = 0.95  # Smoothing factor for low-pass filtering

##########################
# Global Variables
##########################
roll = 0.0
pitch = 0.0
yaw = 0.0

# Filtered gyro values
gyro_x_f = 0.0
gyro_y_f = 0.0
gyro_z_f = 0.0

# For plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Gyroscope Orientation (3D)")
plt.ion()

# We'll store lines/plots of our 3D axes here
obj_lines = []

# Connect to serial
ser = serial.Serial(PORT, BAUD_RATE)

##########################
# Utility Functions
##########################
def euler_to_rotation_matrix(roll, pitch, yaw):
    """Compute rotation matrix from roll, pitch, yaw (in radians)."""
    # Rotation about X-axis
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    # Rotation about Y-axis
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [ 0,             1, 0            ],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Rotation about Z-axis
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])

    # Combined rotation matrix
    # Order of multiplication depends on convention. We'll do Rz * Ry * Rx.
    R = Rz @ Ry @ Rx
    return R


def draw_orientation(roll, pitch, yaw):
    global obj_lines

    # Remove old lines
    for line in obj_lines:
        line.remove()
    obj_lines.clear()

    # Rotation Matrix
    R = euler_to_rotation_matrix(roll, pitch, yaw)

    # We'll plot 3 small axes (length=0.5) to visualize orientation of the object
    axes_length = 0.5

    # Define the axes in the object's local frame
    # X-axis is (0,0,0) -> (axes_length, 0, 0)
    # Y-axis is (0,0,0) -> (0, axes_length, 0)
    # Z-axis is (0,0,0) -> (0, 0, axes_length)
    x_local = np.array([axes_length, 0, 0])
    y_local = np.array([0, axes_length, 0])
    z_local = np.array([0, 0, axes_length])
    origin = np.array([0, 0, 0])

    # Rotate them into global frame
    x_global = R @ x_local
    y_global = R @ y_local
    z_global = R @ z_local

    # Plot each axis using a line from origin to end
    # We'll store the references to these lines in obj_lines so we can remove them on the next update
    line_x, = ax.plot([origin[0], x_global[0]], [origin[1], x_global[1]], [origin[2], x_global[2]], color='r')
    line_y, = ax.plot([origin[0], y_global[0]], [origin[1], y_global[1]], [origin[2], y_global[2]], color='g')
    line_z, = ax.plot([origin[0], z_global[0]], [origin[1], z_global[1]], [origin[2], z_global[2]], color='b')

    obj_lines.extend([line_x, line_y, line_z])


def update_orientation(gx, gy, gz):
    global roll, pitch, yaw
    global gyro_x_f, gyro_y_f, gyro_z_f

    # Simple low-pass filter
    gyro_x_f = SMOOTHING_ALPHA * gyro_x_f + (1.0 - SMOOTHING_ALPHA) * gx
    gyro_y_f = SMOOTHING_ALPHA * gyro_y_f + (1.0 - SMOOTHING_ALPHA) * gy
    gyro_z_f = SMOOTHING_ALPHA * gyro_z_f + (1.0 - SMOOTHING_ALPHA) * gz

    # Integrate to get approximate angles
    roll  += gyro_x_f * DT
    pitch += gyro_y_f * DT
    yaw   += gyro_z_f * DT

    # Keep angles within -pi to pi for neatness (optional)
    roll  = (roll + np.pi) % (2*np.pi) - np.pi
    pitch = (pitch + np.pi) % (2*np.pi) - np.pi
    yaw   = (yaw + np.pi) % (2*np.pi) - np.pi

    # Draw updated orientation
    draw_orientation(roll, pitch, yaw)

# Main loop
try:
    while True:
        # Read data from serial
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        try:
            gx, gy, gz = map(float, line.split(','))
        except ValueError:
            # If the line isn't in the correct format, skip it.
            continue

        # Update orientation
        update_orientation(gx, gy, gz)

        # Update plot
        plt.draw()
        plt.pause(1.0 / UPDATE_RATE)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    plt.ioff()
    plt.show()
