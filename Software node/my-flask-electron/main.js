const { app, BrowserWindow } = require('electron');
const { spawn } = require('child_process');
const path = require('path');

let mainWindow;
let nodeProcess, pythonProcess;

// Helper function to resolve paths relative to the parent folder
function resolveParentPath(fileName) {
    return path.join(app.getAppPath(), '..', fileName);
}

app.on('ready', () => {
    // Start the first program (Node.js server)
    const serverPath = resolveParentPath('server.js');
    nodeProcess = spawn('node', [serverPath]);

    nodeProcess.stdout.on('data', (data) => {
        console.log(`Node.js Server: ${data}`);
    });

    nodeProcess.stderr.on('data', (data) => {
        console.error(`Node.js Server Error: ${data}`);
    });

    nodeProcess.on('close', (code) => {
        console.log(`Node.js server process exited with code ${code}`);
    });

    // Start the second program (Python Flask app)
    const pythonPath = resolveParentPath('app.py');
    pythonProcess = spawn('python', [pythonPath]);

    pythonProcess.stdout.on('data', (data) => {
        console.log(`Flask App: ${data}`);
    });

    pythonProcess.stderr.on('data', (data) => {
        console.error(`Flask App Error: ${data}`);
    });

    pythonProcess.on('close', (code) => {
        console.log(`Flask app process exited with code ${code}`);
    });

    // Wait a few seconds to ensure both programs are up and running
    setTimeout(() => {
        mainWindow = new BrowserWindow({
            width: 1200,
            height: 900,
            webPreferences: {
                nodeIntegration: true,
            },
            autoHideMenuBar: true, // Hides the menu bar
        });

        // Load the Flask app URL
        mainWindow.loadURL('http://127.0.0.1:5000'); // Replace with your Flask app's URL

        // Handle window close event
        mainWindow.on('closed', () => {
            mainWindow = null;
        });
    }, 5000); // Adjust the delay if needed
});

app.on('window-all-closed', () => {
    // Terminate the child processes when the Electron app is closed
    if (nodeProcess) {
        console.log("Terminating Node.js server process...");
        nodeProcess.kill('SIGINT'); // Graceful termination signal
    }

    if (pythonProcess) {
        console.log("Terminating Flask app process...");
        pythonProcess.kill('SIGINT'); // Graceful termination signal
    }

    if (process.platform !== 'darwin') {
        app.quit();
    }
});

app.on('before-quit', () => {
    // Ensure the processes are terminated before the app quits
    if (nodeProcess) {
        nodeProcess.kill('SIGINT');
    }
    if (pythonProcess) {
        pythonProcess.kill('SIGINT');
    }
});