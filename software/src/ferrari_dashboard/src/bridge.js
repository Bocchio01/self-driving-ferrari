#!/usr/bin/env node

const fs = require('fs');
const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');
const express = require('express');
const http = require('http');
const path = require('path');

async function main() {
    await rclnodejs.init();
    const node = rclnodejs.createNode('ferrari_dashboard_bridge');

    const app = express();
    const server = http.createServer(app);

    let publicPath = path.join(__dirname, '..', 'public');
    app.use(express.static(publicPath));

    const wss = new WebSocket.Server({ server });

    wss.on('connection', (ws) => {
        console.log('Browser client connected to dashboard');
        ws.on('close', () => console.log('Browser client disconnected'));
    });

    const qosProfile = new rclnodejs.QoS();
    qosProfile.reliability = rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qosProfile.history = rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qosProfile.depth = 2;

    node.createSubscription(
        'sensor_msgs/msg/CompressedImage',
        'image_raw/compressed',
        { qos: qosProfile },
        (msg) => {
            const rawJpegData = msg.data;
            wss.clients.forEach((client) => {
                if (client.readyState === WebSocket.OPEN) {
                    client.send(rawJpegData);
                }
            });
        }
    );

    const PORT = 8080;
    server.listen(PORT, () => {
        console.log(`\n======================================================`);
        console.log(` Ferrari Dashboard running at: http://localhost:${PORT}`);
        console.log(`======================================================\n`);
    });


    process.on('SIGINT', () => {
        console.log('\nShutting down Ferrari Dashboard...');

        wss.clients.forEach((client) => {
            client.terminate();
        });

        wss.close();
        server.close();
        rclnodejs.shutdown();
        process.exit(0);
    });

    rclnodejs.spin(node);
}

main().catch((err) => console.error('Failed to start bridge:', err));