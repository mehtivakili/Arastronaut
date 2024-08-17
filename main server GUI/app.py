from flask import Flask, request, render_template, jsonify, send_from_directory, send_file
from flask_cors import CORS
import os
import esptool
import serial.tools.list_ports
import socket
import subprocess
import serial
import struct
import threading 
from threading import Thread
from threading import Event
import time
import zlib
import csv
import queue
from flask_socketio import SocketIO
# import eventlet
import socketio

import matplotlib
matplotlib.use('Agg')  # Use a non-GUI backend

import matplotlib.pyplot as plt

import pandas as pd

sio_client = socketio.Client()

data_queue = queue.Queue()
# Allow the server to handle more simultaneous connections
# eventlet.monkey_patch()

app = Flask(__name__)

socketio = SocketIO(app)

message_queue = queue.Queue()

CORS(app)

ESP32_IP = "192.168.4.1"  # IP address of the ESP32
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 12346  # Port to bind the UDP socket
global sock

off1 = 0
off2 = 0
off3 = 0

dist1 = 0
dist2 = 0
dist3 = 0

dist10 = 0
dist20 = 0
dist30 = 0

# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow the socket to reuse the address

# sock.bind((UDP_IP, UDP_PORT))

# # Start the UDP listener in a separate thread
# udp_thread = threading.Thread(target=udp_listener)
# udp_thread.start()

cycle_counter_limit = 2
batch_size = 1

# Calibration parameters
calibration_enabled = False
Ta = [[1, -0.00546066, 0.00101399], [0, 1, 0.00141895], [0, 0, 1]]
Ka = [[0.00358347, 0, 0], [0, 0.00358133, 0], [0, 0, 0.00359205]]
Tg = [[1, -0.00614889, -0.000546488], [0.0102258, 1, 0.000838491], [0.00412113, 0.0020154, 1]]
Kg = [[0.000531972, 0, 0], [0, 0.000531541, 0], [0, 0, 0.000531]]
acce_bias = [-8.28051, -4.6756, -0.870355]
gyro_bias = [4.53855, 4.001, -1.9779]

# calibration_enabled = False
# Ta = [[1, -0.0033593, -0.00890639], [0, 1, -0.0213341], [0, 0, 1]]
# Ka = [[0.00241278, 0, 0], [0, 0.00242712, 0], [0, 0, 0.00241168]]
# Tg = [[1, 0.00593634, 0.00111101], [0.00808812, 1, -0.0535569], [0.0253076, -0.0025513, 1]]
# Kg = [[0.000209295, 0, 0], [0, 0.000209899, 0], [0, 0, 0.000209483]]
# acce_bias = [33124.2, 33275.2, 32364.4]
# gyro_bias = [32777.1, 32459.8, 32511.8]

# Control flag for the UDP listener thread
# udp_thread_running = False
# udp_thread = None

# udp_thread_running2 = False
# udp_thread2 = None

# udp_thread_running3 = False
# udp_thread3 = None

udp_thread_stop_event = Event()
udp_thread = None

udp_thread2_stop_event = Event()
udp_thread2 = None

udp_thread3_stop_event = Event()
udp_thread3 = None

def apply_calibration(accel, gyro):
    acce_calibrated = [
        ((int)(((Ka[0][0] * Ta[0][0]) + (Ka[0][1] * Ta[1][1]) + (Ka[0][2] * Ta[2][2])) * (accel[0] - acce_bias[0]) * 1000)) / 1000.0,
        ((int)(((Ka[1][1] * Ta[1][1]) + (Ka[1][2] * Ta[2][2])) * (accel[1] - acce_bias[1]) * 1000)) / 1000.0,
        ((int)(((Ka[2][2] * Ta[2][2])) * (accel[2] - acce_bias[2]) * 1000)) / 1000.0
    ]
    gyro_calibrated = [
        ((int)(((Kg[0][0] * Tg[0][0]) + (Kg[0][1] * Tg[1][1]) + (Kg[0][2] * Tg[2][2])) * (gyro[0] - gyro_bias[0]) * 1000)) / 1000.0,
        ((int)(((Kg[1][0] * Tg[1][0]) + (Kg[1][1] * Tg[1][1]) + (Kg[1][2] * Tg[2][2])) * (gyro[1] - gyro_bias[1]) * 1000)) / 1000.0,
        ((int)(((Kg[2][0] * Tg[2][0]) + (Kg[2][1] * Tg[2][1]) + (Kg[2][2] * Tg[2][2])) * (gyro[2] - gyro_bias[2]) * 1000)) / 1000.0
    ]
    return acce_calibrated, gyro_calibrated


anchor_positions = {
    130: {"x": 0, "y": 5, "z": 0},
    131: {"x": 5, "y": 0, "z": 0},
    133: {"x": 5, "y": 5, "z": 0},
}

state = "imu"

FIRMWARE_BASE_PATH = 'firmwares'
serial_port = None
serial_thread = None
serial_running = False

def get_network_info():
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    ssid = get_connected_ssid()
    network_info = {
        "hostname": hostname,
        "local_ip": local_ip,
        "ssid": ssid
    }
    return network_info

def get_connected_ssid():
    try:
        result = subprocess.check_output("netsh wlan show interfaces", shell=True).decode()
        for line in result.split("\n"):
            if "SSID" in line:
                ssid = line.split(":")[1].strip()
                return ssid
    except Exception as e:
        print(f"Could not get SSID: {e}")
        return "Unknown"
    
def start_udp_thread(target_function, stop_event):
    global udp_thread, udp_thread2, udp_thread3

    if stop_event.is_set():
        stop_event.clear()  # Clear the event if it was set

    thread = threading.Thread(target=target_function, args=(stop_event,))
    thread.start()
    return thread

def stop_udp_thread(stop_event, thread):
    stop_event.set()  # Signal the thread to stop
    if thread:
        thread.join()  # Wait for the thread to finish


@app.route('/', methods=['GET', 'POST'])
def index():
    global udp_thread_running, udp_thread, udp_thread_running2, udp_thread2, udp_thread_running3, udp_thread3
 
    stop_udp_thread(udp_thread_stop_event, udp_thread)
    stop_udp_thread(udp_thread2_stop_event, udp_thread2)
    stop_udp_thread(udp_thread3_stop_event, udp_thread3)

    network_info = get_network_info()
    return render_template('index.html', network_info=network_info)

    # global udp_thread_running, udp_thread2, state, udp_thread
    # if request.method == 'POST':
    #     # Start the UDP thread
    #     udp_thread_running = True
    #     state = "uwb"
    #     if udp_thread:
    #         udp_thread.join()
    #     udp_thread2 = threading.Thread(target=read_serial_data)
    #     udp_thread2.start()
    #     print("UDP thread2 started")
    # elif request.method == 'POST':
    #     # Stop the UDP thread
    #     udp_thread_running = False
    #     if udp_thread2:
    #         udp_thread2.join()
    #         print("UDP thread2 stopped")

@app.route('/start')
def start():
    try:
        response = requests.get(f'http://{ESP32_IP}/start')
        return jsonify({"status": response.text})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/stop')
def stop():
    try:
        response = requests.get(f'http://{ESP32_IP}/stop')
        return jsonify({"status": response.text})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})


@app.route('/firmware')
def firmware():
    network_info = get_network_info()
    return render_template('firmware.html', network_info=network_info)

# @app.before_request
# def before_request():
#     stop_udp_thread(udp_thread_stop_event, udp_thread)
#     stop_udp_thread(udp_thread2_stop_event, udp_thread2)
#     stop_udp_thread(udp_thread3_stop_event, udp_thread3)
    
@app.route('/data_acquisition', methods=['GET', 'POST'])
def data_acquisition():
    global udp_thread_running, udp_thread, state, udp_thread2, udp_thread3
    stop_udp_thread(udp_thread3_stop_event, udp_thread3)  # Ensure the previous thread is stopped

    if request.method == 'POST':

        udp_thread3 = start_udp_thread(read_serial_data, udp_thread3_stop_event)
        print("UDP thread started")

    # if request.method == 'GET':
    #     # stop_udp_thread(udp_thread3_stop_event, udp_thread3)  # Ensure the previous thread is stopped

    #     udp_thread3 = start_udp_thread(read_serial_data, udp_thread3_stop_event)
    #     print("UDP thread started")

    network_info = get_network_info()
    return render_template('data_acquisition.html', network_info=network_info)

@app.route('/calibration')
def calibration():
    global udp_thread_running, udp_thread, udp_thread_running2, udp_thread2, udp_thread_running3, udp_thread3
 
    stop_udp_thread(udp_thread_stop_event, udp_thread)
    stop_udp_thread(udp_thread2_stop_event, udp_thread2)
    stop_udp_thread(udp_thread3_stop_event, udp_thread3)

    network_info = get_network_info()
    return render_template('calibration_page.html', network_info=network_info)

# @app.route('/calibration_page')
# def calibration():
#     network_info = get_network_info()
#     return render_template('calibration.html', network_info=network_info)

# @app.route('/imu_calibration')
# def imu_calibration():
#     network_info = get_network_info()
#     return render_template('imu_calibration.html', network_info=network_info)


# @app.route('/uwb_udp')
# def uwb_udp():
#     network_info = get_network_info()
#     # Start the UDP listener in a separate thread
#     udp_thread = threading.Thread(target=uwb_udp_listener)
#     udp_thread.start()
#     print("UDP thread started")

#     return render_template('uwb_udp.html', network_info=network_info)

@app.route('/uwb_calibration', methods=['GET', 'POST'])
def uwb_udp():
    global udp_thread_running, udp_thread2, state, udp_thread, udp_thread3
    stop_udp_thread(udp_thread3_stop_event, udp_thread3)  # Ensure the previous thread is stopped

    if request.method == 'POST':
        # Start the UDP thread
            

        udp_thread3 = start_udp_thread(uwb_udp_listener, udp_thread3_stop_event)
        print("UDP thread2 started")
    # elif request.method == 'POST':
    #     # Stop the UDP thread
    #     udp_thread_running = False
    #     if udp_thread2:
    #         udp_thread2.join()
    #         print("UDP thread2 stopped")
    
    network_info = get_network_info()
    return render_template('uwb_calibration.html', network_info=network_info)

@app.route('/set_anchor_positions', methods=['POST'])
def set_anchor_positions():
    data = request.get_json()
    address = data.get('address')
    x = data.get('x')
    y = data.get('y')
    z = data.get('z')

    if address in anchor_positions and x is not None and y is not None and z is not None:
        try:
            x = float(x)
            y = float(y)
            z = float(z)
            anchor_positions[address] = {"x": x, "y": y, "z": z}
            return jsonify(status='success', anchor=anchor_positions[address])
        except ValueError:
            return "Invalid coordinates", 400
    else:
           return "Invalid address or missing coordinates", 400



@app.route('/magnometer_calibration')
def magnometer_calibration():
    network_info = get_network_info()
    return render_template('magnometer_calibration.html', network_info=network_info)

# @app.route('/uwb_calibration')
# def uwb_calibration():
#     network_info = get_network_info()
#     return render_template('uwb_calibration.html', network_info=network_info)

# @app.route('/uwb_precalib_param', methods= ['POST'])
# def uwb_precalib_param():
#     global 

@app.route('/update_cycle_counter', methods=['POST'])
def update_cycle_counter():
    global cycle_counter_limit
    data = request.json
    cycle_counter_limit = data.get('cycleCounter', cycle_counter_limit)
    return jsonify({'status': 'success', 'cycleCounter': cycle_counter_limit})

@app.route('/update_batch_size', methods=['POST'])
def update_batch_size():
    global batch_size
    data = request.json
    batch_size = data.get('batchSize', batch_size)
    return jsonify({'status': 'success', 'batchSize': batch_size})

@app.route('/device_orientation', methods=['POST'])
def device_orientation():
    global udp_thread_running, udp_thread, state, udp_thread2, udp_thread3
    stop_udp_thread(udp_thread3_stop_event, udp_thread3)  # Ensure the previous thread is stopped

    if request.method == 'POST':

        udp_thread3 = start_udp_thread(read_serial_data, udp_thread3_stop_event)
        print("UDP thread started")
    network_info = get_network_info
    return render_template('device_orientation.html', network_info=network_info)




@app.route('/python_serial', methods=['GET', 'POST'])
# @app.route('/python_serial')
def python_serial():
    if request.method == 'POST':
        # try:
        #     port = request.form['port']
        #     baudrate = int(request.form['baudrate'])
        #     is_binary = request.form.get('is_binary', 'false') == 'true'
        #     open_serial_port(port, baudrate, is_binary)
        #     return jsonify(status='success')
        # except KeyError as e:
        #     return jsonify(status='error', message=f"Missing form key: {str(e)}")
        # except Exception as e:
        #     return jsonify(status='error', message=str(e))
    # else:
        network_info = get_network_info()
        return render_template('python_serial.html', network_info=network_info)

# @app.route('/python_UDP', methods=['GET', 'POST'])
# def python_UDP():
#     global serial_thread
#     if request.method == 'POST':
#             # Start the UDP thread
#             serial_thread = threading.Thread(target=read_serial_data)
#             serial_thread.start()
#             print("UDP thread started")
 
#             network_info = get_network_info()
#             return render_template('python_UDP.html', network_info=network_info)
@app.route('/imu_calibration', methods=['GET', 'POST'])
def imu_calib():
    global udp_thread_running, udp_thread, state, udp_thread2, udp_thread3
    if request.method == 'POST':
        stop_udp_thread(udp_thread2_stop_event, udp_thread2)  # Ensure the previous thread is stopped

        udp_thread2 = start_udp_thread(read_serial_data, udp_thread2_stop_event)
        print("IMU calib thread started")
 
        # stop_udp_thread(udp_thread_stop_event, udp_thread)
 
        # udp_thread = start_udp_thread(read_serial_data, udp_thread_stop_event)
        # udp_thread.start()
        # print("UDP thread started")
 
    
    network_info = get_network_info()
    return render_template('imu_calibration.html', network_info=network_info)

# @app.route('/device-orientation', methods=['GET', 'POST'])
# def device_orientation():
#         if(request.content_type == POST):
#             udp_thread = threading.Thread(target=read_serial_data)
#             udp_thread.start()
#         print("UDP thread started")
#     network_info = get_network_info()
#     return render_template('python_UDP.html', network_info=network_info)

@app.route('/get_ports', methods=['GET'])

def get_ports():
    ports = list(serial.tools.list_ports.comports())
    port_list = [{'device': port.device, 'description': port.description} for port in ports]
    return jsonify(port_list)

@app.route('/flash', methods=['POST'])
def flash():
    port = request.form['port']
    baudrate = request.form['baudrate']
    group = request.form['group']


    folder_path = os.path.join(FIRMWARE_BASE_PATH, group)
    print(folder_path)
    if not os.path.exists(folder_path):
        return 'Invalid group selected'

    bootloader_path = os.path.join(folder_path, 'bootloader.bin')
    partitions_path = os.path.join(folder_path, 'partitions.bin')
    firmware_path = os.path.join(folder_path, 'firmware.bin')

    

    flash_firmware(port, baudrate, bootloader_path, partitions_path, firmware_path)
    
    return 'Firmware flashed successfully'

def flash_firmware(port, baudrate, bootloader_path, partitions_path, firmware_path):
    esptool_args = [
        '--chip', 'esp32',
        '--port', port,
        '--baud', baudrate,
        'write_flash',
        '0x1000', bootloader_path,
        '0x8000', partitions_path,
        '0x10000', firmware_path
    ]
    esptool.main(esptool_args)
    
@app.route('/open_serial', methods=['POST'])
def open_serial():
        
    try:
        port = request.form['port']
        baudrate = int(request.form['baudrate'])
        is_binary = request.form.get('is_binary', 'false') == 'true'
        open_serial_port(port, baudrate, is_binary)
        return jsonify(status='success')
    except KeyError as e:
        return jsonify(status='error', message=f"Missing form key: {str(e)}")
    except Exception as e:
        return jsonify(status='error', message=str(e))

def open_serial_port(port, baudrate, is_binary):
    global serial_port, serial_thread, serial_running

    if serial_port and serial_port.is_open:
        serial_port.close()
    # sss.emit("data_rate","aaaa")
    # print(sss)
    # print("ashfuiahf")
    serial_port = serial.Serial(port, baudrate, timeout=1)
    serial_running = True
    serial_thread = threading.Thread(target=read_serial_data, args=(is_binary,))
    serial_thread.start()



def calculate_checksum(data):
    return sum(data) & 0xFF

offset = 0
from datetime import datetime

def calculate_tag_position(dist1, dist2, dist3):
    # Trilateration logic to calculate tag position based on distances from anchors
    x1, y1, z1 = anchor_positions[130]['x'], anchor_positions[130]['y'], anchor_positions[130]['z']
    x2, y2, z2 = anchor_positions[131]['x'], anchor_positions[131]['y'], anchor_positions[131]['z']
    x3, y3, z3 = anchor_positions[133]['x'], anchor_positions[133]['y'], anchor_positions[133]['z']

    # Example simplified 2D trilateration
    A = 2 * x2 - 2 * x1
    B = 2 * y2 - 2 * y1
    C = dist1**2 - dist2**2 - x1**2 + x2**2 - y1**2 + y2**2
    D = 2 * x3 - 2 * x2
    E = 2 * y3 - 2 * y2
    F = dist2**2 - dist3**2 - x2**2 + x3**2 - y2**2 + y3**2

    x = (C * E - F * B) / (E * A - B * D)
    y = (C * D - A * F) / (B * D - A * E)
    z = 0  # Assuming 2D plane

    return {"x": x, "y": y, "z": z}




def uwb_udp_listener(stop_event2):

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow the socket to reuse the address

    sock.bind((UDP_IP, UDP_PORT))

    last_received_time = time.time()
    visible_device = False

    buffer = bytearray()
    UWB_SEPARATOR= b'cba/'

    lock = threading.Lock()

    global off1, off2, off3
    global dist1, dist2, dist3
    global dist10, dist20, dist30
    
    while not stop_event2.is_set():
        # with lock:
        #     off1_ = off1
        #     off2_ = off2
        #     off3_ = off3
        try:
            sock.settimeout(10)

            data, addr = sock.recvfrom(4096)
            buffer.extend(data)
            # print(f"Buffer length: {len(buffer)}")

            current_time = time.time()

            while len(buffer) >= 16:
                start_index = buffer.find(UWB_SEPARATOR)
                if start_index == -1:
                    break  # UWB separator not found, wait for more data

                end_index = start_index + len(UWB_SEPARATOR) + 12  # 12 bytes for the UWB data
                if end_index > len(buffer):
                    break  # Not enough data for a full UWB packet, wait for more data

                part = buffer[start_index + len(UWB_SEPARATOR):end_index]
                buffer = buffer[end_index:]  # Remove the processed part from the buffer

                if len(part) == 12:  # 12 bytes: 4 for Tio, 2 for address, 4 for distance
                    # Tio, address, dist = struct.unpack('<3f', part)

                    Tio = struct.unpack('f', data[4:8])[0]
                    address = struct.unpack('f', data[8:12])[0]
                    distance = struct.unpack('f', data[12:16])[0]


                    # print(off1_, off2_, off3_)

                    uwb_data1 = {
                        'time': Tio,
                        'short_address': address,
                        'distance': distance - off1,
                        'visible': True
                    }
                    
                    uwb_data2 = {
                        'time': Tio,
                        'short_address': address,
                        'distance': distance - off2,
                        'visible': True
                    }
                    
                    uwb_data3 = {
                        'time': Tio,
                        'short_address': address,
                        'distance': distance - off3,
                        'visible': True
                    }

                    if address == 130:
                        # print(uwb_data1)
                        dist1 = distance
                        dist10 = distance - off1
                        sio_client.emit('uwb_data', uwb_data1)                
                    elif address == 131:
                        # print(uwb_data2)
                        dist2 = distance
                        dist20 = distance -off2
                        sio_client.emit('uwb_data', uwb_data2)                
                    elif address == 133:
                        # print(uwb_data3)
                        dist3 = distance
                        dist30 = distance - off3
                        sio_client.emit('uwb_data', uwb_data3)

                    # Calculate tag position once all distances are available
                    if dist10 > 0 and dist20 > 0 and dist30 > 0:
                        tag_position = calculate_tag_position(dist10, dist20, dist30)
                        # print(dist10, dist20, dist30)
                        print(tag_position)
                        sio_client.emit('tag_position', tag_position)


                    # uwb_data = {
                    #     'time': Tio,
                    #     'short_address': address,
                    #     'distance': distance,
                    #     'visible': True
                    # }

                    # uwb_data = {
                    #     'time': "tio",
                    #     'short_address': "short_address",
                    #     'distance': "distance",
                    #     'visible': True
                    # }
                    # print(uwb_data)
                    # sio_client.emit('uwb_data', uwb_data)
                    # socketio.emit('uwb_data', uwb_data)
                    last_received_time = current_time
                    visible_device = True
                    # print("uwb data sent!!!!!!!")
                elif visible_device and (current_time - last_received_time > 3):
                    uwb_data = {
                        'visible': False
                    }
                    # sio2_client.emit('uwb_data', uwb_data)

                    socketio.emit('uwb_data', uwb_data)
                    visible_device = False
        except socket.timeout:
            continue

    sock.close()
    print("UDP listener thread2 terminated.")

@app.route('/set_uwb_offset', methods=['POST'])
def set_uwb_offset():
    global off1, off2, off3
    global dist1, dist2, dist3

    # Get the address and offset from the form data
    data = request.get_json()
    address = data.get('address')
    offset = data.get('offset')

    print(address, offset)

    if address is not None and offset is not None:
        try:
            # Convert offset to the appropriate type, if necessary
            offset = float(offset)
            address = float(address)  # Ensure address is an integer

            
            # Now you can use `address` and `offset` as needed
            print(f"Received Address: {address}, Offset: {offset}")

            

            if address == 130:
                off1 = dist1 - offset
                print(off1)
            elif address == 131:
                off2 = dist2 - offset
            elif address == 133:
                off3 = dist3 - offset

            # Here you can implement your logic to set the offset
            # For example, update global variables or trigger calibration
            global set_offset, calibration_offset, calibration_address
            calibration_address = address
            calibration_offset = offset
            set_offset = True

            return jsonify(status='success')
        except ValueError:
            return "Invalid offset value", 400
    else:
        return "Missing address or offset", 400

def read_serial_data(stop_event):
    print("read_serial_data started")
    global serial_running
    global last_Tio 
    global packets_count
    global current_second_start
    global offset
    global Timer
    global count
    global set_offset
    global most_recent_acc_file
    global most_recent_gyro_file
    global calibration_enabled
    global numbers
    global batch_size
    global cycle_counter_limit
    global udp_thread_running
    global state
    # global sock
    numbers = []
    # offset = 0  # Set this to your required offset
    last_Tio = 0
    # Timer = 10000
    current_second_start = 0
    packets_count = 0
    set_offset = False
    start_time =0
    end_time = 0
    time_IO = 0
    cycle_counter = 0  # Counter to keep track of cycles
    mag_counter = 0
    # Send the 'c' character to request data
    # serial_port.write(check.encode())
    # with open('data_log.csv', mode='a', newline='') as file:
    #     writer = csv.writer(file)
    timestamp = datetime.now().strftime("%Y%m%d%H%M")
    acc_filename = f"acc-{timestamp}.csv"
    gyro_filename = f"gyro-{timestamp}.csv"
    mag_filename = f"mag-{timestamp}.csv"
    imu_uwb_filename = f"imu_uwb-{timestamp}.csv"
    global createdFlag
    createdFlag = True

    # Update the most recent filenames
    most_recent_acc_file = acc_filename
    most_recent_gyro_file = gyro_filename

    # most_recent_acc_file = "test_imu_acc.calib"
    # most_recent_gyro_file = "test_imu_gyro.calib"
    # batch_size = 10
    batch_data = []
    check = "abc/"
    check_encoded = check.encode()
    check_length = len(check_encoded)

    UWB_SEPARATOR = b'cba/'  # Assuming 'uwb/' is the separator for UWB data
    UWB_PACKET_SIZE = 16  # 4 bytes for Tio, 2 bytes for address, 4 bytes for distance, and 4 bytes for separator
    visible_device = False
    address_ = "null"

    buffer = bytearray()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow the socket to reuse the address

    sock.bind((UDP_IP, UDP_PORT))
    # while serial_running:
    #     if serial_port.in_waiting > 0:
   
 #         buffer.extend(serial_port.read(serial_port.in_waiting))
    address = 0
    dist = 0

    global dist1, dist2, dist3

    add = []

    global off1, off2, off3


    
    contor_uwb_view = 0

    print("why the ..")
    time.sleep(5)
    while not stop_event.is_set():
        # if(state == "imu"):
        # if(True):
        try:
            sock.settimeout(10)
            data, addr = sock.recvfrom(4096)
            buffer.extend(data)
            print(f"Buffer length: {len(buffer)}")

            # Process UWB data
            while len(buffer) >= UWB_PACKET_SIZE:
                start_index = buffer.find(UWB_SEPARATOR)
                if start_index == -1:
                    break  # UWB separator not found, wait for more data

                end_index = start_index + len(UWB_SEPARATOR) + 12  # 12 bytes for the UWB data
                if end_index > len(buffer):
                    break  # Not enough data for a full UWB packet, wait for more data

                part = buffer[start_index + len(UWB_SEPARATOR):end_index]
                buffer = buffer[end_index:]  # Remove the processed part from the buffer

                if len(part) == 12:  # 12 bytes: 4 for Tio, 2 for address, 4 for distance
                    # Tio, address, dist = struct.unpack('<3f', part)
                    
                    Tio = struct.unpack('f', data[4:8])[0]
                    address = struct.unpack('f', data[8:12])[0]
                    distance = struct.unpack('f', data[12:16])[0]


                    if address == 130:
                        sio_client.emit('UWB_data', {'Tio': Tio, 'address': address, 'distance': dist + off1})
                    elif address == 131:
                        sio_client.emit('UWB_data', {'Tio': Tio, 'address': address, 'distance': dist + off2})
                    elif address == 133:
                        sio_client.emit('UWB_data', {'Tio': Tio, 'address': address, 'distance': dist + off3})

                    # print(f"UWB Data - Tio: {Tio:.3f}, Address: {address}, Distance: {dist:.2f}")
                    # sio_client.emit('UWB_data', {'Tio': Tio, 'address': address, 'distance': dist})
                    


            while len(buffer) >= 32:  # 4 bytes for the "abc/" and 28 bytes for the packet
                start_index = buffer.find(check_encoded)
                if start_index == -1:
                    break  # "abc/" not found, wait for more data


                end_index = start_index + len(check_encoded) + 28
                if end_index > len(buffer):
                    break  # Not enough data for a full packet, wait for more data

                part = buffer[start_index + len(check_encoded):end_index]
                buffer = buffer[end_index:]  # Remove the processed part from the buffer

                if len(part) == 28:   # 28 bytes for the packet (Tio accx accy accz gyrox gyroy gyroz) 4*7 + (magx magy magz) 4*3 = 40
                    numbers = struct.unpack('<7f', part)
                    # Tio, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = numbers
                    # print(f"Tio: {Tio:.3f}, Accel: ({accelX:.2f}, {accelY:.2f}, {accelZ:.2f}), Gyro: ({gyroX:.2f}, {gyroY:.2f}, {gyroZ:.2f})")
                    
                    if not set_offset:
                        offset = numbers[0]
                        set_offset = True
                    
                    # Extract data
                    Tio = numbers[0] - offset
                    accel = numbers[1:4]
                    gyro = numbers[4:7]
                    # mag = numbers[7:10]

                    print (f"Tio: {Tio:.3f}, Accel: {accel}, Gyro: {gyro}")
                    # if(address != 0):
                        # print(f"Address: {address}, Distance: {dist}, Tio: {Tio:.3f}, Accel: {accel}, Gyro: {gyro}")
                    # print(f"Tio: {Tio}, Accel: {accel}, Gyro: {gyro}")
                    if calibration_enabled:
                        accel, gyro = apply_calibration(accel, gyro)
                    # print(address)
                    # if(address != 0):


                    # if(address == 130):
                    #     dist1 = dist
                    # if(address == 131):
                    #     dist2 = dist
                    # if(address == 133):
                    #     dist3 = dist

                # # Dynamic addressing logic
                # if address not in add:
                #     add.append(address)
                #     if len(add) == 3:
                #         addSet = True
                #     elif len(add) > 3:
                #         add = add[:3]  # Ensure only 3 unique addresses are stored

                # # Store distances based on address
                # if addSet:
                #     if address == add[0]:
                #         dist1 = dist
                #     elif address == add[1]:
                #         dist2 = dist
                #     elif address == add[2]:
                #         dist3 = dist


                    contor_uwb_view = contor_uwb_view + 1
                    if contor_uwb_view > 100:
                        # print(f"{dist1:.2e}", f"{dist2:.2e}", f"{dist3:.2e}")
                        contor_uwb_view = 0
                    # Initialize the first Tio and current second start
                    if last_Tio is None:
                        last_Tio = Tio
                        current_second_start = int(Tio)

                    # Increment the cycle counter
                    cycle_counter += 1
                    mag_counter += 1

                    # if mag_counter %30 == 0:
                    #             formatted_mag = [f"{mag[0]:.7e}", f"{mag[1]:.7e}", f"{mag[2]:.7e}"]

                    #             with open(mag_filename, mode='a', newline='') as mag_file:
                    #                 mag_writer = csv.writer(mag_file, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
                    #                 mag_writer.writerow(formatted_mag)

                    # Emit data every 20 cycles
                    if cycle_counter >= cycle_counter_limit:
                        # Append data to batch
                        batch_data.append({'Tio': Tio, 'accel': accel, 'gyro': gyro})
                        if len(batch_data) >= batch_size:
                            # Emit the batch of data
                            sio_client.emit('sensor_data', batch_data)

                            # Reset the batch data
                            batch_data = []
                        # Reset the cycle counter
                        cycle_counter = 0

                    # Check for Tio change and calculate rate
                    if int(Tio) != current_second_start:
                        # Print the rate for the last second
                        # print(f"Tio: {Tio}, Data rate: {packets_count} packets in the last second")

                        # Reset the packet count for the new second
                        packets_count = 0
                        current_second_start = int(Tio)

                    # Increment the packet count for the current second
                    packets_count += 1
                    last_Tio = Tio
                    if Timer != 0:
                        if start_time != 0:
                            end_time = time.time()
                            if (end_time - start_time < Timer):
                                formatted_accel = [f"{Tio:.7e}", f"{accel[0]:.7e}", f"{accel[1]:.7e}", f"{accel[2]:.7e}"]
                                formatted_gyro = [f"{Tio:.7e}", f"{gyro[0]:.7e}", f"{gyro[1]:.7e}", f"{gyro[2]:.7e}"]

                                with open(acc_filename, mode='a', newline='') as acc_file:
                                    acc_writer = csv.writer(acc_file, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
                                    acc_writer.writerow(formatted_accel)
                                with open(gyro_filename, mode='a', newline='') as gyro_file:
                                    gyro_writer = csv.writer(gyro_file, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
                                    gyro_writer.writerow(formatted_gyro)
                                # formatted_imu_uwb = [f"{Tio:.7e}", f"{accel[0]:.7e}", f"{accel[1]:.7e}", f"{accel[2]:.7e}", f"{gyro[0]:.7e}", f"{gyro[1]:.7e}", f"{gyro[2]:.7e}", f"{address:.3e}, {dist:.4e}"]
                                # formatted_imu_uwb = [f"{Tio:.7e}", f"{accel[0]:.7e}", f"{accel[1]:.7e}", f"{accel[2]:.7e}", f"{gyro[0]:.7e}", f"{gyro[1]:.7e}", f"{gyro[2]:.7e}", f"{dist1:.3e}", f"{dist2:.3e}", f"{dist3:.3e}"]

                                # print(formatted_imu_uwb)
                                # with open(imu_uwb_filename, mode='a', newline='') as imu_uwb_file:
                                #     imu_uwb_writer = csv.writer(imu_uwb_file, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
                                #     imu_uwb_writer.writerow(formatted_imu_uwb)
                                # createdFlag = True

                            else:
                                if createdFlag:
                                    print(f"{acc_filename} and {gyro_filename} are created.")
                                    # print(f"{imu_uwb_filename} is created.")

                                    createdFlag = False
                        else:
                            start_time = time.time()

                    # Remove the processed packet and the separator from the buffer
                    # buffer = buffer[packet_end:]
                else:
                    print(f"Expected 28 bytes but received {len(parts)} bytes.")
                    break

        except socket.timeout:
            continue
        # elif (state == "uwb"):
                    # data, addr = sock.recvfrom(4096)
            current_time = time.time()

            # if data.startswith(b'cba/'):
            if udp_thread_running:
                # tio = struct.unpack('f', data[4:8])[0]
                # short_address = struct.unpack('f', data[8:12])[0]
                # distance = struct.unpack('f', data[12:16])[0]

                # uwb_data = {
                #     'time': tio,
                #     'short_address': short_address,
                #     'distance': distance,
                #     'visible': True
                # }
                uwb_data = {
                    'time': "tio",
                    'short_address': "short_address",
                    'distance': "distance",
                    'visible': True
                }
                # print(uwb_data)
                # sio_client.emit('uwb_data', uwb_data)
                socketio.emit('uwb_data', uwb_data)
                last_received_time = current_time
                visible_device = True
                # print("uwb data sent!!!!!!!")
            elif visible_device and (current_time - last_received_time > 3):
                uwb_data = {
                    'visible': False
                }
                sio_client.emit('uwb_data', uwb_data)

                # socketio.emit('uwb_data', uwb_data)
                visible_device = False


    # else:
    sock.close()
    print("UDP listener thread terminated.")

        # break

Timer = 0

# def send_data():
#     global count
#     while True:
#         if count != 0:
#             print(count)
#         else:
#             print("No data to send")
#             time.sleep(1)

def start_client():
    sio_client.connect('http://localhost:3000')  # Connect to Node.js server on port 3000
    sio_client.wait()


@app.route('/start_recording', methods=['POST'])
def start_recording():
    print('haha')
    
    global offset, Timer, acc_filename, gyro_filename, createdFlag
    Timer = float(request.form['offset'])
    if True:
        # serial_port.write(b'c')
        # time.sleep(0.1)
        # data = serial_port.read(28)
        if True:
            # numbers = struct.unpack('<7f', data)
            createdFlag = True
            offset = numbers[0]
            timestamp = datetime.now().strftime("%Y%m%d%H%M")
            acc_filename = f"acc-{timestamp}.csv"
            gyro_filename = f"gyro-{timestamp}.csv"


            # Update the most recent filenames
            most_recent_acc_file = acc_filename
            most_recent_gyro_file = gyro_filename

            # print(offset)
            return jsonify({'status': 'success'})
        else:
            return jsonify({'status': 'error', 'message': 'Failed to read offset'})
    return jsonify({'status': 'error', 'message': 'Serial port not opened'})

count = 0

@app.route('/close_serial', methods=['POST'])
def close_serial():
    global serial_port, serial_running, set_offset
    set_offset = False
    serial_running = False
    if serial_port and serial_port.is_open:
        serial_port.close()
    return jsonify(status='success')

@app.route('/stop_recording', methods=['GET'])
def stop_recording():
    print("hahahasdfafafads")
    # Implement your stop recording logic here
        # Delete the most recent files
    if most_recent_acc_file and os.path.exists(most_recent_acc_file):
        os.remove(most_recent_acc_file)
        print(f"Deleted file: {most_recent_acc_file}")
    if most_recent_gyro_file and os.path.exists(most_recent_gyro_file):
        os.remove(most_recent_gyro_file)
        print(f"Deleted file: {most_recent_gyro_file}")
    return jsonify(status='successfully stopped')


@app.route('/plot_data', methods=['GET'])
def plot_data():
    global most_recent_acc_file, most_recent_gyro_file
    # recording_name = request.args.get('recordingName')
    # if not recording_name:
    #     return jsonify(status='error', message='Recording name not provided'), 400

    # # Construct file paths based on recording name
    # acc_file_path = most_recent_acc_file
    # gyro_file_path = most_recent_gyro_file

    try:
        # Call the plotting script as a subprocess
        result = subprocess.run(
            ["python", "plot_script.py", most_recent_acc_file, most_recent_gyro_file],
            capture_output=True,
            text=True
        )

        if result.returncode != 0:
            # Log the stderr output for debugging
            print(f"Error: {result.stderr.strip()}")
            return jsonify(status='error', message=result.stderr.strip()), 500

        plot_file_path = result.stdout.strip()

        return send_file(plot_file_path, mimetype='image/png')
    except FileNotFoundError as e:
        return jsonify(status='error', message=str(e)), 404

import requests  # Ensure you have requests module installed

@app.route('/calibrate', methods=['GET'])
def calibrate():
    # most_recent_acc_file ="xsens_acc.mat"
    # most_recent_gyro_file = "xsens_gyro.mat"
    if most_recent_acc_file and most_recent_gyro_file:
        command = f"./test_imu_calib {most_recent_acc_file} {most_recent_gyro_file}"
        try:
            child = pexpect.spawn(command)
            for _ in range(3):
                child.sendline("")  # Send "Enter" key press
                time.sleep(3)  # Wait for 2 seconds
            child.expect(pexpect.EOF)
            output = child.before.decode('utf-8')

            # Read the calibration files
            try:
                with open("test_imu_acc.calib", "r") as acc_calib_file:
                    acc_calib_data = acc_calib_file.read()
                with open("test_imu_gyro.calib", "r") as gyro_calib_file:
                    gyro_calib_data = gyro_calib_file.read()
                
                calib_params = {
                    'acc_calib': acc_calib_data,
                    'gyro_calib': gyro_calib_data
                }
                
                # Return the calibration data as a JSON response
                return jsonify({'status': 'success', 'output': output, 'calib_params': calib_params})
                
                if response.status_code == 200:
                    return jsonify({'status': 'success', 'output': output, 'server_response': response.json()})
                else:
                    return jsonify({'status': 'error', 'message': 'Failed to send calibration data to the server'})
                
            except Exception as e:
                return jsonify({'status': 'error', 'message': f'Failed to read calibration files: {str(e)}'})
            

            # return jsonify({'status': 'success', 'output': output})
        except pexpect.ExceptionPexpect as e:
            return jsonify({'status': 'error', 'message': str(e)})
    else:
        return jsonify({'status': 'error', 'message': 'No recent files to calibrate'})

@app.route('/get_calibrated_data', methods=['GET'])
def get_calibrated_data():
    global calibration_enabled
    calibration_enabled = True
    return jsonify(status='success', message='Calibrated data will now be applied')

import re

def parse_calibration_data(data):
    lines = data.strip().split('\n')
    parsed_data = []
    for line in lines:
        numbers = re.findall(r'[-+]?\d*\.\d+|\d+', line)
        if numbers:
            parsed_data.append([float(num) for num in numbers])
    return parsed_data

@app.route('/upload_calibration_files', methods=['POST'])
def upload_calibration_files():
    data = request.json
    acc_data = data['accData']
    gyro_data = data['gyroData']

    try:
        global Ta, Ka, acce_bias
        global Tg, Kg, gyro_bias

        acc_parsed = parse_calibration_data(acc_data)
        gyro_parsed = parse_calibration_data(gyro_data)

        if len(acc_parsed) < 9 or len(gyro_parsed) < 9:
            raise ValueError("Incomplete calibration data")

        Ta = acc_parsed[:3]
        Ka = acc_parsed[3:6]
        acce_bias = [row[0] for row in acc_parsed[6:9]]

        Tg = gyro_parsed[:3]
        Kg = gyro_parsed[3:6]
        gyro_bias = [row[0] for row in gyro_parsed[6:9]]
        print(Ta, Ka, acce_bias)
        print(Tg, Kg, gyro_bias)

        return jsonify(status='success', message='Calibration parameters updated')
    except Exception as e:
        print(f"Error: {str(e)}")
        return jsonify(status='error', message=str(e)), 400


import pexpect

# @app.route('/receive_calib_params', methods=['POST'])
# def receive_calib_params():
#     data = request.json
#     if 'acc_calib' in data and 'gyro_calib' in data:
#         acc_calib = data['acc_calib']
#         gyro_calib = data['gyro_calib']
#         print("Received Accelerometer Calibration Data:", acc_calib)
#         print("Received Gyroscope Calibration Data:", gyro_calib)
#         return jsonify({'status': 'success', 'message': 'Calibration data received'})
#     else:
#         return jsonify({'status': 'error', 'message': 'Invalid calibration data'}), 400


# @socketio.on('connect')
# def handle_connect(socket):
#     # while not message_queue.empty():
#     #     print("oomad")   
#     # print(socket)
#     socketio.emit('connect', {'data': 'Connected'})

# Handle connection event for the client


@sio_client.event
def connect():
    print('Client connection established')
    sio_client.send('Hello from Flask Client!')

# Handle message event for the client
@sio_client.event
def message(data):
    print('Message received by client: ', data)

# Handle disconnection event for the client
@sio_client.event
def disconnect():
    print('Client disconnected from server')


if __name__ == "__main__":
    # Start the Socket.IO client in a separate thread
    client_thread = threading.Thread(target=start_client)
    client_thread.start()
    socketio.run(app, debug=True)

        

