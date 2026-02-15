/**
 * Main Application
 * 
 * Initializes all modules and orchestrates the AR Robot Controller.
 */

// ============================================
// Global instances
// ============================================
let gestureRecognizer;
let rosBridge;
let joystick;
let lidarViz;

let controlMode = 'gesture';  // 'gesture' or 'joystick'
let isEmergencyStopped = false;
let showHandOverlay = true;
let maxSpeed = 0.5;

// Hand drawing canvas
const handCanvas = document.getElementById('hand-canvas');
const handCtx = handCanvas.getContext('2d');

// ============================================
// Initialization
// ============================================
async function init() {
    console.log('🚀 Initializing AR Robot Controller...');

    // Resize canvases
    resizeCanvases();
    window.addEventListener('resize', resizeCanvases);

    // Initialize ROS Bridge
    rosBridge = new ROSBridge();

    // If running inside Android app, get ROS URL from AndroidBridge
    let rosUrl;
    if (typeof AndroidBridge !== 'undefined' && AndroidBridge.getROSBridgeURL) {
        rosUrl = AndroidBridge.getROSBridgeURL();
        document.getElementById('ros-url').value = rosUrl;
    } else {
        rosUrl = document.getElementById('ros-url').value;
    }
    rosBridge.connect(rosUrl);

    rosBridge.onConnectionChange = (connected) => {
        updateROSStatus(connected);
    };

    rosBridge.onStatus((data) => {
        updateVelocityDisplay(data);
    });

    rosBridge.onLidar((data) => {
        if (lidarViz) lidarViz.updateData(data);
    });

    // Initialize LiDAR Visualizer
    lidarViz = new LidarVisualizer('lidar-canvas');

    // Initialize Gesture Recognizer
    gestureRecognizer = new GestureRecognizer();
    const video = document.getElementById('camera-feed');

    try {
        await gestureRecognizer.initialize(video);
        updateHandStatus(true);
    } catch (error) {
        console.error('Failed to initialize hand tracking:', error);
        updateHandStatus(false);
        // Fall back to joystick mode
        setMode('joystick');
    }

    // Set up gesture callbacks
    gestureRecognizer.onGesture((data) => {
        if (controlMode === 'gesture') {
            handleGesture(data);
        }
    });

    gestureRecognizer.onLandmarks((landmarks, handedness) => {
        drawHandLandmarks(landmarks, handedness);
    });

    // Initialize Virtual Joystick
    joystick = new VirtualJoystick('joystick-container');
    joystick.onJoystickMove((x, y) => {
        if (controlMode === 'joystick' && !isEmergencyStopped) {
            rosBridge.publishJoystick(x, y);
        }
    });

    // Show gesture guide on first visit
    if (!localStorage.getItem('ar_controller_visited')) {
        toggleGuide();
        localStorage.setItem('ar_controller_visited', 'true');
    }

    console.log('✅ AR Robot Controller ready!');
}

// ============================================
// Gesture Handling
// ============================================
function handleGesture(data) {
    // Update UI
    updateGestureDisplay(data.gesture, data.confidence);
    updateHandStatus(data.landmarks !== null);

    if (data.confidence < 0.7) return;

    // Send gesture to ROS
    rosBridge.publishGesture({
        gesture: data.gesture,
        confidence: data.confidence,
        speed_factor: maxSpeed * data.speedFactor,
        pinch_distance: data.pinchDistance
    });
}

// ============================================
// Hand Landmark Drawing
// ============================================
function drawHandLandmarks(landmarks, handedness) {
    handCtx.clearRect(0, 0, handCanvas.width, handCanvas.height);

    if (!landmarks || !showHandOverlay) return;

    // Define finger connections
    const connections = [
        // Thumb
        [0, 1], [1, 2], [2, 3], [3, 4],
        // Index
        [0, 5], [5, 6], [6, 7], [7, 8],
        // Middle
        [0, 9], [9, 10], [10, 11], [11, 12],
        // Ring
        [0, 13], [13, 14], [14, 15], [15, 16],
        // Pinky
        [0, 17], [17, 18], [18, 19], [19, 20],
        // Palm
        [5, 9], [9, 13], [13, 17]
    ];

    const w = handCanvas.width;
    const h = handCanvas.height;

    // Draw connections
    handCtx.strokeStyle = 'rgba(0, 200, 255, 0.7)';
    handCtx.lineWidth = 3;

    for (const [i, j] of connections) {
        const a = landmarks[i];
        const b = landmarks[j];
        handCtx.beginPath();
        handCtx.moveTo(a.x * w, a.y * h);
        handCtx.lineTo(b.x * w, b.y * h);
        handCtx.stroke();
    }

    // Draw landmarks
    for (let i = 0; i < landmarks.length; i++) {
        const lm = landmarks[i];
        const x = lm.x * w;
        const y = lm.y * h;
        const isTip = [4, 8, 12, 16, 20].includes(i);

        handCtx.beginPath();
        handCtx.arc(x, y, isTip ? 8 : 5, 0, Math.PI * 2);
        handCtx.fillStyle = isTip ? 'rgba(255, 100, 100, 0.9)' : 'rgba(0, 200, 255, 0.8)';
        handCtx.fill();

        if (isTip) {
            handCtx.beginPath();
            handCtx.arc(x, y, 12, 0, Math.PI * 2);
            handCtx.strokeStyle = 'rgba(255, 100, 100, 0.4)';
            handCtx.lineWidth = 2;
            handCtx.stroke();
        }
    }
}

// ============================================
// UI Updates
// ============================================
const gestureEmoji = {
    'NONE': '🤖',
    'FIST': '✊ STOP',
    'INDEX_POINT': '☝️ FORWARD',
    'TWO_FINGERS': '✌️ BACKWARD',
    'OPEN_HAND': '🛑 E-STOP',
    'THUMB_LEFT': '👈 LEFT',
    'THUMB_RIGHT': '👉 RIGHT',
    'PINCH': '🤏 SPEED',
};

function updateGestureDisplay(gesture, confidence) {
    const display = document.getElementById('gesture-display');
    const text = gestureEmoji[gesture] || gesture;
    const conf = Math.round(confidence * 100);
    display.textContent = `${text} (${conf}%)`;

    // Color based on gesture
    if (gesture === 'OPEN_HAND') {
        display.style.borderColor = 'rgba(239, 68, 68, 0.5)';
        display.style.color = '#fca5a5';
    } else if (gesture === 'NONE') {
        display.style.borderColor = 'rgba(255, 255, 255, 0.1)';
        display.style.color = '#888';
    } else {
        display.style.borderColor = 'rgba(0, 200, 255, 0.5)';
        display.style.color = '#fff';
    }
}

function updateROSStatus(connected) {
    const dot = document.getElementById('ros-status');
    const text = document.getElementById('ros-status-text');
    if (connected) {
        dot.className = 'status-dot connected';
        text.textContent = 'ROS Connected';
    } else {
        dot.className = 'status-dot';
        text.textContent = 'Disconnected';
    }
}

function updateHandStatus(detected) {
    const dot = document.getElementById('hand-status');
    const text = document.getElementById('hand-status-text');
    if (detected) {
        dot.className = 'status-dot connected';
        text.textContent = 'Hand Detected';
    } else {
        dot.className = 'status-dot warning';
        text.textContent = 'No Hand';
    }
}

function updateVelocityDisplay(data) {
    document.getElementById('linear-vel').textContent = 
        `${(data.linear_vel || 0).toFixed(3)} m/s`;
    document.getElementById('angular-vel').textContent = 
        `${(data.angular_vel || 0).toFixed(3)} rad/s`;
}

// ============================================
// Control Functions
// ============================================
function setMode(mode) {
    controlMode = mode;
    document.getElementById('mode-display').textContent = 
        mode === 'gesture' ? 'Gesture' : 'Joystick';

    document.getElementById('btn-gesture').classList.toggle('active', mode === 'gesture');
    document.getElementById('btn-joystick').classList.toggle('active', mode === 'joystick');

    if (mode === 'joystick') {
        joystick.show();
    } else {
        joystick.hide();
    }
}

function toggleEmergencyStop() {
    isEmergencyStopped = !isEmergencyStopped;
    const btn = document.getElementById('emergency-stop');

    if (isEmergencyStopped) {
        btn.classList.add('stopped');
        btn.textContent = 'RESUME';
        rosBridge.publishGesture({ gesture: 'EMERGENCY_STOP', confidence: 1.0, speed_factor: 0 });
    } else {
        btn.classList.remove('stopped');
        btn.textContent = 'E-STOP';
        rosBridge.publishGesture({ gesture: 'RESUME', confidence: 1.0, speed_factor: 0 });
    }
}

function toggleSettings() {
    document.getElementById('settings-panel').classList.toggle('show');
}

function toggleGuide() {
    document.getElementById('gesture-guide').classList.toggle('show');
}

function reconnectROS() {
    const url = document.getElementById('ros-url').value;
    rosBridge.connect(url);
}

function updateMaxSpeed(val) {
    maxSpeed = parseFloat(val);
    document.getElementById('speed-val').textContent = val;
}

function toggleLidar() {
    const enabled = document.getElementById('show-lidar').checked;
    lidarViz.toggle(enabled);
}

function toggleHand() {
    showHandOverlay = document.getElementById('show-hand').checked;
}

// ============================================
// Canvas Resize
// ============================================
function resizeCanvases() {
    handCanvas.width = window.innerWidth;
    handCanvas.height = window.innerHeight;
}

// ============================================
// Start Application
// ============================================
window.addEventListener('load', init);
