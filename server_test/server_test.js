const net = require('net');
const os = require('os');
const Speaker = require('speaker');

const server = net.createServer(socket => {
    console.log('Client connected');

    let speaker;

    socket.on('error', error => {
        console.error('Socket error: ', error);
        speaker.end();
    });

    socket.on('data', (data) => {
        if (!speaker) {
            speaker = new Speaker({
                channels: 1,
                bitDepth: 16,
                sampleRate: 16000
            });
        }

        speaker.write(data);
    });


    socket.on('end', () => {
        console.log('Client disconnected');
        if (speaker) {
            speaker.end();
        }
    });
});

server.listen(3030, () => {
    console.log('Server is listening on port 3030');

    const interfaces = os.networkInterfaces();
    const addresses = [];
    for (const k in interfaces) {
        for (const k2 in interfaces[k]) {
            const address = interfaces[k][k2];
            if (address.family === 'IPv4' && !address.internal) {
                addresses.push(address.address);
            }
        }
    }

    console.log('Server IP addresses: ', addresses);
});
