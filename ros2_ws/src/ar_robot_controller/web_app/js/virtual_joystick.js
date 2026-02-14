/**
 * Virtual Joystick Module
 * 
 * Touch-based virtual joystick as an alternative 
 * to hand gesture control. Works on mobile devices.
 */

class VirtualJoystick {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.base = document.getElementById('joystick-base');
        this.handle = document.getElementById('joystick-handle');
        
        this.active = false;
        this.x = 0;  // -1 to 1
        this.y = 0;  // -1 to 1
        this.maxRadius = 60;
        this.onMoveCallback = null;

        this.centerX = 0;
        this.centerY = 0;

        this.bindEvents();
    }

    bindEvents() {
        // Touch events
        this.base.addEventListener('touchstart', (e) => this.onStart(e.touches[0]), { passive: false });
        this.base.addEventListener('touchmove', (e) => {
            e.preventDefault();
            this.onMove(e.touches[0]);
        }, { passive: false });
        this.base.addEventListener('touchend', () => this.onEnd());
        this.base.addEventListener('touchcancel', () => this.onEnd());

        // Mouse events (for desktop testing)
        this.base.addEventListener('mousedown', (e) => this.onStart(e));
        document.addEventListener('mousemove', (e) => {
            if (this.active) this.onMove(e);
        });
        document.addEventListener('mouseup', () => {
            if (this.active) this.onEnd();
        });
    }

    onStart(event) {
        this.active = true;
        this.handle.classList.add('active');
        
        const rect = this.base.getBoundingClientRect();
        this.centerX = rect.left + rect.width / 2;
        this.centerY = rect.top + rect.height / 2;

        this.onMove(event);
    }

    onMove(event) {
        if (!this.active) return;

        const clientX = event.clientX || event.pageX;
        const clientY = event.clientY || event.pageY;

        let dx = clientX - this.centerX;
        let dy = clientY - this.centerY;

        // Clamp to max radius
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > this.maxRadius) {
            dx = (dx / dist) * this.maxRadius;
            dy = (dy / dist) * this.maxRadius;
        }

        // Normalize to -1 to 1
        this.x = dx / this.maxRadius;
        this.y = -dy / this.maxRadius;  // Invert Y (up = positive)

        // Update handle position
        this.handle.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

        if (this.onMoveCallback) {
            this.onMoveCallback(this.x, this.y);
        }
    }

    onEnd() {
        this.active = false;
        this.x = 0;
        this.y = 0;
        this.handle.classList.remove('active');
        this.handle.style.transform = 'translate(-50%, -50%)';

        if (this.onMoveCallback) {
            this.onMoveCallback(0, 0);
        }
    }

    onJoystickMove(callback) {
        this.onMoveCallback = callback;
    }

    show() {
        this.container.style.display = 'block';
    }

    hide() {
        this.container.style.display = 'none';
        this.onEnd();
    }
}
