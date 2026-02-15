/**
 * Gesture Recognizer Module
 * 
 * Uses MediaPipe Hands to detect hand landmarks
 * and classify gestures for robot control.
 * 
 * Gestures:
 *   FIST         - All fingers closed
 *   INDEX_POINT  - Only index finger extended
 *   TWO_FINGERS  - Index + middle finger extended
 *   OPEN_HAND    - All fingers extended
 *   THUMB_LEFT   - Thumb pointing left
 *   THUMB_RIGHT  - Thumb pointing right
 *   PINCH        - Thumb and index finger close together
 */

class GestureRecognizer {
    constructor() {
        this.hands = null;
        this.camera = null;
        this.landmarks = null;
        this.gesture = 'NONE';
        this.confidence = 0;
        this.pinchDistance = 0;
        this.speedFactor = 0.5;
        this.isInitialized = false;
        this.onGestureCallback = null;
        this.onLandmarksCallback = null;

        // Finger tip and pip landmark indices
        this.FINGER_TIPS = [4, 8, 12, 16, 20];    // thumb, index, middle, ring, pinky
        this.FINGER_PIPS = [3, 6, 10, 14, 18];     // PIP joints
        this.FINGER_MCPS = [2, 5, 9, 13, 17];      // MCP joints

        // Gesture smoothing
        this.gestureHistory = [];
        this.historySize = 5;
    }

    async initialize(videoElement) {
        this.hands = new Hands({
            locateFile: (file) => {
                return `https://cdn.jsdelivr.net/npm/@mediapipe/hands@0.4/${file}`;
            }
        });

        this.hands.setOptions({
            maxNumHands: 1,
            modelComplexity: 1,
            minDetectionConfidence: 0.7,
            minTrackingConfidence: 0.5
        });

        this.hands.onResults((results) => this.processResults(results));

        this.camera = new Camera(videoElement, {
            onFrame: async () => {
                await this.hands.send({ image: videoElement });
            },
            width: 640,
            height: 480
        });

        await this.camera.start();
        this.isInitialized = true;
        console.log('✋ GestureRecognizer initialized');
    }

    processResults(results) {
        if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
            this.landmarks = results.multiHandLandmarks[0];
            this.confidence = results.multiHandedness[0].score;

            const gesture = this.classifyGesture(this.landmarks);
            this.gesture = this.smoothGesture(gesture);

            if (this.onGestureCallback) {
                this.onGestureCallback({
                    gesture: this.gesture,
                    confidence: this.confidence,
                    landmarks: this.landmarks,
                    speedFactor: this.speedFactor,
                    pinchDistance: this.pinchDistance
                });
            }

            if (this.onLandmarksCallback) {
                this.onLandmarksCallback(this.landmarks, results.multiHandedness[0]);
            }
        } else {
            this.landmarks = null;
            this.gesture = 'NONE';
            this.confidence = 0;

            if (this.onGestureCallback) {
                this.onGestureCallback({
                    gesture: 'NONE',
                    confidence: 0,
                    landmarks: null,
                    speedFactor: 0,
                    pinchDistance: 0
                });
            }

            if (this.onLandmarksCallback) {
                this.onLandmarksCallback(null, null);
            }
        }
    }

    classifyGesture(landmarks) {
        const fingers = this.getFingerStates(landmarks);
        const thumbDir = this.getThumbDirection(landmarks);

        // Count extended fingers
        const extendedCount = fingers.filter(f => f).length;

        // Calculate pinch distance (thumb tip to index tip)
        this.pinchDistance = this.distance(landmarks[4], landmarks[8]);

        // PINCH: thumb and index very close
        if (this.pinchDistance < 0.06) {
            this.speedFactor = 0.0;
            return 'PINCH';
        }

        // Update speed factor based on pinch distance
        this.speedFactor = Math.min(1.0, Math.max(0.1, this.pinchDistance * 3));

        // OPEN_HAND: all 5 fingers extended
        if (extendedCount >= 5) {
            return 'OPEN_HAND';
        }

        // FIST: no fingers extended (or just thumb)
        if (extendedCount <= 0) {
            return 'FIST';
        }

        // INDEX_POINT: only index finger extended
        if (fingers[1] && !fingers[2] && !fingers[3] && !fingers[4]) {
            return 'INDEX_POINT';
        }

        // TWO_FINGERS: index + middle extended
        if (fingers[1] && fingers[2] && !fingers[3] && !fingers[4]) {
            return 'TWO_FINGERS';
        }

        // THUMB direction (only thumb extended)
        if (fingers[0] && !fingers[1] && !fingers[2] && !fingers[3] && !fingers[4]) {
            if (thumbDir === 'left') return 'THUMB_LEFT';
            if (thumbDir === 'right') return 'THUMB_RIGHT';
        }

        // Default
        return 'NONE';
    }

    getFingerStates(landmarks) {
        const states = [];

        // Thumb: compare tip x to IP joint x (considering hand orientation)
        const wrist = landmarks[0];
        const thumbTip = landmarks[4];
        const thumbIP = landmarks[3];
        const thumbMCP = landmarks[2];

        // Determine if right or left hand based on wrist-to-thumb direction
        const isRightHand = thumbMCP.x < landmarks[5].x;

        if (isRightHand) {
            states.push(thumbTip.x < thumbIP.x);
        } else {
            states.push(thumbTip.x > thumbIP.x);
        }

        // Other fingers: compare tip y to PIP y (lower y = more extended in image coords)
        for (let i = 1; i < 5; i++) {
            const tip = landmarks[this.FINGER_TIPS[i]];
            const pip = landmarks[this.FINGER_PIPS[i]];
            states.push(tip.y < pip.y);
        }

        return states;
    }

    getThumbDirection(landmarks) {
        const thumbTip = landmarks[4];
        const thumbMCP = landmarks[2];
        const wrist = landmarks[0];

        const dx = thumbTip.x - thumbMCP.x;
        const dy = thumbTip.y - thumbMCP.y;

        // Primarily horizontal movement
        if (Math.abs(dx) > Math.abs(dy) * 1.5) {
            return dx > 0 ? 'right' : 'left';
        }

        return 'neutral';
    }

    smoothGesture(gesture) {
        this.gestureHistory.push(gesture);
        if (this.gestureHistory.length > this.historySize) {
            this.gestureHistory.shift();
        }

        // Find most common gesture in history
        const counts = {};
        let maxCount = 0;
        let maxGesture = gesture;

        for (const g of this.gestureHistory) {
            counts[g] = (counts[g] || 0) + 1;
            if (counts[g] > maxCount) {
                maxCount = counts[g];
                maxGesture = g;
            }
        }

        // Only switch if majority agrees
        return maxCount >= Math.ceil(this.historySize / 2) ? maxGesture : this.gesture;
    }

    distance(a, b) {
        return Math.sqrt(
            Math.pow(a.x - b.x, 2) + 
            Math.pow(a.y - b.y, 2) + 
            Math.pow((a.z || 0) - (b.z || 0), 2)
        );
    }

    onGesture(callback) {
        this.onGestureCallback = callback;
    }

    onLandmarks(callback) {
        this.onLandmarksCallback = callback;
    }

    destroy() {
        if (this.camera) {
            this.camera.stop();
        }
    }
}
