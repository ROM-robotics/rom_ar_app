/**
 * ROS Bridge Module
 * 
 * Handles WebSocket connection to rosbridge_server
 * and provides ROS2 topic pub/sub interface.
 */

class ROSBridge {
    constructor() {
        this.ros = null;
        this.connected = false;
        this.url = 'ws://localhost:9090';

        // Publishers
        this.gesturePub = null;
        this.joystickPub = null;

        // Subscribers
        this.statusSub = null;
        this.lidarSub = null;

        // Callbacks
        this.onStatusCallback = null;
        this.onLidarCallback = null;
        this.onConnectionChange = null;

        // Reconnection
        this.reconnectTimer = null;
        this.reconnectInterval = 3000;
    }

    connect(url) {
        if (url) this.url = url;

        if (this.ros) {
            this.ros.close();
        }

        this.ros = new ROSLIB.Ros({ url: this.url });

        this.ros.on('connection', () => {
            console.log('🔌 Connected to rosbridge:', this.url);
            this.connected = true;
            this.setupTopics();
            if (this.onConnectionChange) this.onConnectionChange(true);
            if (this.reconnectTimer) {
                clearInterval(this.reconnectTimer);
                this.reconnectTimer = null;
            }
        });

        this.ros.on('error', (error) => {
            console.error('❌ ROS connection error:', error);
        });

        this.ros.on('close', () => {
            console.log('🔌 Disconnected from rosbridge');
            this.connected = false;
            if (this.onConnectionChange) this.onConnectionChange(false);
            this.startReconnect();
        });
    }

    startReconnect() {
        if (!this.reconnectTimer) {
            this.reconnectTimer = setInterval(() => {
                console.log('🔄 Attempting to reconnect...');
                this.connect();
            }, this.reconnectInterval);
        }
    }

    setupTopics() {
        // Publisher: gesture commands
        this.gesturePub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/ar_controller/gesture',
            messageType: 'std_msgs/String'
        });

        // Publisher: joystick data
        this.joystickPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/ar_controller/joystick',
            messageType: 'std_msgs/Float32MultiArray'
        });

        // Subscriber: controller status
        this.statusSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/ar_controller/status',
            messageType: 'std_msgs/String'
        });

        this.statusSub.subscribe((msg) => {
            try {
                const data = JSON.parse(msg.data);
                if (this.onStatusCallback) this.onStatusCallback(data);
            } catch (e) {
                console.error('Failed to parse status:', e);
            }
        });

        // Subscriber: LiDAR viz data
        this.lidarSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/ar_controller/lidar_viz',
            messageType: 'std_msgs/String'
        });

        this.lidarSub.subscribe((msg) => {
            try {
                const data = JSON.parse(msg.data);
                if (this.onLidarCallback) this.onLidarCallback(data);
            } catch (e) {
                console.error('Failed to parse lidar data:', e);
            }
        });
    }

    publishGesture(gestureData) {
        if (!this.connected || !this.gesturePub) return;

        const msg = new ROSLIB.Message({
            data: JSON.stringify(gestureData)
        });
        this.gesturePub.publish(msg);
    }

    publishJoystick(x, y) {
        if (!this.connected || !this.joystickPub) return;

        const msg = new ROSLIB.Message({
            layout: {
                dim: [],
                data_offset: 0
            },
            data: [x, y]
        });
        this.joystickPub.publish(msg);
    }

    onStatus(callback) {
        this.onStatusCallback = callback;
    }

    onLidar(callback) {
        this.onLidarCallback = callback;
    }

    disconnect() {
        if (this.reconnectTimer) {
            clearInterval(this.reconnectTimer);
        }
        if (this.ros) {
            this.ros.close();
        }
    }
}
