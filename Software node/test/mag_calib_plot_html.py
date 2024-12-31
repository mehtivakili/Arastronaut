from flask import Flask, render_template, Response, make_response
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import threading
import io

app = Flask(__name__)

# UDP configuration
rate = 0
UDP_IP = "0.0.0.0"
UDP_PORT = 12346
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow the socket to reuse the address

sock.bind((UDP_IP, UDP_PORT))

# Data storage
magnetometer_data = []
offsets = np.array([0.0, 0.0, 0.0])

# Lock for thread safety
data_lock = threading.Lock()

def remove_outliers_and_calibrate(data):
    if len(data) == 0:
        return np.array([0.0, 0.0, 0.0]), data
    
    data = np.array(data)
    mean = np.mean(data, axis=0)
    std_dev = np.std(data, axis=0)
    
    z_scores = np.abs((data - mean) / std_dev)
    data_filtered = data[(z_scores < 2).all(axis=1)]
    
    offsets = np.mean(data_filtered, axis=0)
    data_corrected = data_filtered - offsets
    
    return offsets, data_corrected

def receive_udp_data():
    global rate
    print("UDP thread started.")
    check = "img/"
    check_encoded = check.encode()

    while True:
        data, addr = sock.recvfrom(4096)
        parts = data.split(check_encoded)
        for part in parts:
            if len(part) == 40:
                rate += 1
                values = struct.unpack('<10f', part)
                _, _, _, _, _, _, _, magX, magY, magZ = values
                if rate % 10 == 0:
                    with data_lock:
                        magnetometer_data.append([magX, magY, magZ])
                    print(f"Received data: {magX}, {magY}, {magZ}")

def update_plot():
    global offsets, magnetometer_data
    
    with data_lock:
        if len(magnetometer_data) == 0:
            print("No data to plot.")
            return None  # Don't create a plot if there's no data

        # Calibrate data
        offsets, calibrated_data = remove_outliers_and_calibrate(magnetometer_data)

    # Create a new plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    raw_data = np.array(magnetometer_data)
    if len(raw_data) > 0:
        ax.scatter(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2], c='r', marker='o', label='Raw Data')
    if len(calibrated_data) > 0:
        ax.scatter(calibrated_data[:, 0], calibrated_data[:, 1], calibrated_data[:, 2], c='g', marker='^', label='Calibrated Data')
    
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('Real-Time Magnetometer Data')
    ax.legend()

    # Save the plot to a file for debugging purposes
    fig.savefig('debug_plot.png')
    print("Plot saved to debug_plot.png")

    # Save the plot to a BytesIO object
    buf = io.BytesIO()
    plt.savefig(buf, format='png')
    plt.close(fig)  # Close the figure to free memory
    buf.seek(0)
    print("Plot generated.")
    return buf

@app.route('/')
def index():
    return render_template('index2.html')

@app.route('/plot.png')
def plot_png():
    buf = update_plot()
    if buf is None:
        return "No data available", 204  # Return a "No Content" status if there's no data
    
    response = make_response(buf.read())
    response.headers['Content-Type'] = 'image/png'
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response

if __name__ == "__main__":
    udp_thread = threading.Thread(target=receive_udp_data, daemon=True)
    udp_thread.start()
    app.run(debug=True)
