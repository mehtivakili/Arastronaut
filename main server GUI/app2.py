from flask import Flask, request, render_template, jsonify, send_from_directory
from flask_cors import CORS
import os
import esptool
# from flask_socketio import SocketIO, emit
import serial.tools.list_ports
import socket
import subprocess
import serial
import struct
import threading
import time
import zlib
import csv
import asyncio
import websockets
import json
import queue


# Create lists to hold the data for plotting
Tio_data = []
accel_data = [[], [], []]
gyro_data = [[], [], []]

# Queue for thread-safe communication between serial reading and plotting
data_queue = queue.Queue()

app = Flask(__name__)
CORS(app)
app.config['SECRET_KEY'] = 'secret!'
# socketio = SocketIO(app)

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

@app.route('/')
def index():
    network_info = get_network_info()
    return render_template('index.html', network_info=network_info)

@app.route('/firmware')
def firmware():
    network_info = get_network_info()
    return render_template('firmware.html', network_info=network_info)

@app.route('/data_acquisition')
def data_acquisition():
    network_info = get_network_info()
    return render_template('data_acquisition.html', network_info=network_info)

@app.route('/calibration')
def calibration():
    network_info = get_network_info()
    return render_template('calibration.html', network_info=network_info)

@app.route('/python_serial', methods=['GET', 'POST'])
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

    serial_port = serial.Serial(port, baudrate, timeout=1)
    serial_running = True
    serial_thread = threading.Thread(target=read_serial_data, args=(is_binary,))
    serial_thread.start()


def calculate_checksum(data):
    return sum(data) & 0xFF

# def read_serial_data(is_binary):
#     data = serial_port.read(28)
#     numbers = struct.unpack('<7f', data)
#     global serial_running
#     while serial_running:
#         if serial_port.in_waiting >= 28:  # Wait for the full packet
#             data = serial_port.read(28)
#             numbers = struct.unpack('<7f', data)
#             # Extract data
#             Tio, = struct.unpack('f', data[0:4])
#             accel = struct.unpack('fff', data[4:16])
#             gyro = struct.unpack('fff', data[16:28])
#             # received_crc, = struct.unpack('I', data[32:36])
            
#             # Calculate CRC32 to verify data integrity
#             # calculated_crc = zlib.crc32(data[0:32]) & 0xFFFFFFFF
            
#             # if received_crc == calculated_crc:
#             message = f'Time: {Tio}, Accel: {accel}, Gyro: {gyro}'
#             # else:
#                 # message = 'CRC mismatch, invalid data received.'
            
#             socketio.emit('serial_data', {'data': message})
#         # time.sleep(0.05)

offset = 0
numbers = []
def read_serial_data(true):

    global serial_running
    global last_Tio 
    global packets_count
    global current_second_start
    global offset
    global Timer
    global socketio
    global ani

    # offset = 0  # Set this to your required offset
    last_Tio = 0
    current_second_start = 0
    packets_count = 0
    start_time =0
    end_time = 0
    # Send the 'c' character to request data
    check = 'c'
    serial_port.write(check.encode())
    set_offset = False
    # with open('data_log.csv', mode='a', newline='') as file:
    #     writer = csv.writer(file)
    socket_count = 0

    while serial_running:
        # if serial_port.in_waiting >= 29:  # Wait for the full packet
        #     first_byte = serial_port.read(1)  # Read the first byte
        #     if first_byte == check.encode():  # Check if it matches 'c'
        if serial_port.in_waiting >= 0:  # Wait for the full packet
            first_byte = serial_port.read(1)  # Read the first byte
            if first_byte == b'c':  # Check if it matches 'c'
                data = serial_port.read(28)  # Read the remaining 28 bytes
                if len(data) == 28:
                    socket_count = socket_count + 1
                    global numbers
                    numbers = struct.unpack('<7f', data)  # Unpack the 7 floats
                    if ( set_offset != True) :
                        offset = numbers[0]
                        set_offset = True
                    # Extract data
                    Tio = numbers[0] - offset
                    accel = numbers[1:4]
                    gyro = numbers[4:7]

                    message = f'Time: {Tio}, Accel: {accel}, Gyro: {gyro}'
                    if (socket_count == 19):
                        socket_count = 0
                        # Send data to the client
                        # socketio.emit('update_data', {
                        #     'Tio': Tio,
                        #     'accel': accel,
                        #     'gyro': gyro
                        # })
                        # socketio.emit('update_data', {'Tio': Tio, 'accel': accel, 'gyro': gyro})
                        data_queue.put((Tio, accel, gyro))
                        


                    # Initialize the first Tio and current second start
                    if last_Tio is None:
                        last_Tio = Tio
                        current_second_start = int(Tio)

                    # Check for Tio change and calculate rate
                    if int(Tio) != current_second_start:
                        # Print the rate for the last second
                        print(f"Tio: {Tio}, Data rate: {packets_count} packets in the last second")
                        print(offset)
                        # Reset the packet count for the new second
                        packets_count = 0
                        current_second_start = int(Tio)
                    
                    # Increment the packet count for the current second
                    packets_count += 1
                    last_Tio = Tio

                    # Manage Timer and Start Time for data collection
                    if Timer != 0:
                        if start_time == 0:
                            start_time = time.time()
                        if (time.time() - start_time) <= Timer:
                            # Open the CSV file in append mode
                            with open('data_log.csv', mode='a', newline='') as file:
                                writer = csv.writer(file)
                                # Save data to CSV
                                writer.writerow([Tio, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]])
                        else:
                            # Reset Timer and start_time after collecting data
                            Timer = 0
                            start_time = 0
                    

                else:
                    print(f"Expected 28 bytes but received {len(data)} bytes.")
            else:
                print("First byte did not match expected 'c' character.")
                print(struct.unpack('B', first_byte)[0])
                # Handle unexpected first byte case if needed

Timer = 0

@app.route('/start_recording', methods=['POST'])
def start_recording():
    print('haha')
    global offset, Timer
    Timer = float(request.form['offset'])
    if True:
        # serial_port.write(b'c')
        # time.sleep(0.1)
        # data = serial_port.read(28)
        if True:
            # numbers = struct.unpack('<7f', data)
            offset = numbers[0]
            # print(offset)
            return jsonify({'status': 'success'})
        else:
            return jsonify({'status': 'error', 'message': 'Failed to read offset'})
    return jsonify({'status': 'error', 'message': 'Serial port not opened'})


async def websocket_handler(websocket, path):
    while True:
        print("madarsag")
        try:
            Tio, accel, gyro = data_queue.get_nowait()
            message = {
                'Tio': Tio,
                'accel': accel,
                'gyro': gyro
            }
            await websocket.send(json.dumps(message))
        except queue.Empty:
            await asyncio.sleep(0.1)

@app.route('/plot')
def plot():
    return render_template('plot.html')

def start_websocket_server():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    start_server = websockets.serve(websocket_handler, "localhost", 8765)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()

if __name__ == "__main__":
    threading.Thread(target=start_websocket_server).start()

    app.run(debug=True)