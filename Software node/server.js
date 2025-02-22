const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const cors = require('cors');

// Create a new Express application
const app = express();

// Enable CORS for all routes
app.use(cors());

// Create an HTTP server and pass the Express app to it
const server = http.createServer(app);

// Attach Socket.IO to the HTTP server
const io = socketIo(server, {
  cors: {
    origin: "http://127.0.0.1:5000",
    methods: ["GET", "POST"]
  }
});

// Serve the index.html file at the root URL
app.get('/', (req, res) => {
  res.sendFile(__dirname + '/index.html');
});

// Handle connection events
io.on('connection', (socket) => {
  console.log(`A user connected: ${socket.id}`);
  socket.on('disconnect', () => {
    console.log(`A user disconnected: ${socket.id}`);
  });
  // Handle 'message' events
  socket.on('message', (msg) => {
    console.log('message: ' + msg);
    io.emit('message', msg); // Broadcast the message to all clients
  });

  // Handle 'sensor_data' events from the Flask server
  socket.on('sensor_data', (data) => {
    console.log('sensor_data received: ', data);
    io.emit('sensor_data', data); // Broadcast the sensor data to all clients
  });

  socket.on('uwb_data', (data)=>{
    console.log("UWB_data received: " , data);
    io.emit("uwb_data", data);
  })

  socket.on('tag_position', (data)=>{
    console.log("UWB poistion received: ", data);
    io.emit("tag_position", data);
  })

  socket.on('data_rate', (data) =>{
    console.log("data rate is "+ data);
    io.emit("data_rate", data);
  })

  // Handle disconnection events
  socket.on('disconnect', () => {
    console.log('user disconnected');
  });
});

// Start the server on port 3000
server.listen(3000, () => {
  console.log('listening on *:3000');
});
