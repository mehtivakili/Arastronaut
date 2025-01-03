import subprocess
import threading

def start_node():
    print("Starting Node.js server...")
    subprocess.run("node server.js", shell=True)

def start_python():
    print("Running Python script...")
    subprocess.run("python app.py", shell=True)

# Create threads for parallel execution
node_thread = threading.Thread(target=start_node)
python_thread = threading.Thread(target=start_python)

# Start threads
node_thread.start()
python_thread.start()

# Wait for both threads to complete
node_thread.join()
python_thread.join()
