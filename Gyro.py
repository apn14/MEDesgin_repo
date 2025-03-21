import serial
import matplotlib.pyplot as plt
from collections import deque

# Configuration
PORT = "COM7"  # Replace with your Arduino's COM port
BAUD_RATE = 115200
MAX_POINTS = 100  # Number of points to display on the plot

# Initialize serial connection
ser = serial.Serial(PORT, BAUD_RATE)

# Create data buffers
x_data = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
y_data = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
z_data = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)

# Initialize the plot
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots()
line_x, = ax.plot(x_data, label="Gyro X", color="r")
line_y, = ax.plot(y_data, label="Gyro Y", color="g")
line_z, = ax.plot(z_data, label="Gyro Z", color="b")
plt.legend()
plt.title("Real-Time Gyroscope Data")
plt.xlabel("Time")
plt.ylabel("Angular Velocity (rad/s)")

def update_plot():
    try:
        # Read data from serial
        line = ser.readline().decode("utf-8").strip()
        gyro_x, gyro_y, gyro_z = map(float, line.split(","))
        
        # Update data buffers
        x_data.append(gyro_x)
        y_data.append(gyro_y)
        z_data.append(gyro_z)
        
        # Update plot data
        line_x.set_ydata(x_data)
        line_y.set_ydata(y_data)
        line_z.set_ydata(z_data)
        ax.relim()  # Recalculate limits
        ax.autoscale_view()  # Rescale plot
        plt.draw()  # Redraw the plot
    except ValueError:
        pass  # Ignore malformed data

# Real-time plotting loop
try:
    while True:
        update_plot()
        plt.pause(0.01)  # Pause to allow the plot to update
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    plt.ioff()
