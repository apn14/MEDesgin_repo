import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Configuration
PORT = "COM7"  # Replace with your Arduino's COM port
BAUD_RATE = 115200
UPDATE_INTERVAL = 100  # Milliseconds
ARROW_LENGTH = 5  # Set a constant length for the arrow
NOISE_THRESHOLD = 2  # Magnitude threshold to filter out noise

# Initialize serial connection
ser = serial.Serial(PORT, BAUD_RATE)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Set up plot limits and labels
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([-10, 10])
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.set_title("Real-Time Gyroscope Visualization")

def normalize_vector_to_constant_length(vector, length=5):
    """Normalizes a vector to a constant length."""
    magnitude = np.linalg.norm(vector)
    if magnitude == 0:
        return np.array([0.0, 0.0, 0.0])  # Avoid division by zero
    return (vector / magnitude) * length

def update_plot(gyro_x, gyro_y, gyro_z):
    # Compute the magnitude of the vector
    magnitude = np.linalg.norm([gyro_x, gyro_y, gyro_z])

    # Ignore noise by filtering out small magnitudes
    if magnitude < NOISE_THRESHOLD:
        # Clear the plot but don't show any dynamic arrow
        ax.cla()
        ax.quiver(0, 0, 0, 1, 0, 0, color="r", label="X Axis")  # Static X-axis
        ax.quiver(0, 0, 0, 0, 1, 0, color="g", label="Y Axis")  # Static Y-axis
        ax.quiver(0, 0, 0, 0, 0, 1, color="b", label="Z Axis")  # Static Z-axis
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_xlabel("X Axis")
        ax.set_ylabel("Y Axis")
        ax.set_zlabel("Z Axis")
        ax.legend()
        ax.set_title("Real-Time Gyroscope Visualization")
        plt.draw()
        return

    # Normalize the vector to a constant length for visualization
    constant_length_gyro = normalize_vector_to_constant_length([gyro_x, gyro_y, gyro_z], ARROW_LENGTH)

    # Clear the previous plot and draw updated vectors
    ax.cla()
    ax.quiver(0, 0, 0, 1, 0, 0, color="r", label="X Axis")  # Static X-axis
    ax.quiver(0, 0, 0, 0, 1, 0, color="g", label="Y Axis")  # Static Y-axis
    ax.quiver(0, 0, 0, 0, 0, 1, color="b", label="Z Axis")  # Static Z-axis

    # Dynamic gyroscope vector
    ax.quiver(0, 0, 0, constant_length_gyro[0], constant_length_gyro[1], constant_length_gyro[2], color="black", label="Gyroscope")

    # Set plot limits and labels
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.legend()
    ax.set_title("Real-Time Gyroscope Visualization")
    plt.draw()

try:
    plt.ion()  # Enable interactive mode
    while True:
        # Read data from serial
        line = ser.readline().decode("utf-8").strip()
        try:
            gyro_x, gyro_y, gyro_z = map(float, line.split(","))
            update_plot(gyro_x, gyro_y, gyro_z)
            plt.pause(UPDATE_INTERVAL / 1000.0)  # Pause to allow for updates
        except ValueError:
            # Ignore malformed data
            pass
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    plt.ioff()
