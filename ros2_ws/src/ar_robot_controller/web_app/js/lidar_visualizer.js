/**
 * LiDAR Visualizer Module
 * 
 * Renders LiDAR scan data as a point cloud overlay
 * on the camera feed, creating an AR effect where
 * obstacle distances are visible around the robot.
 */

class LidarVisualizer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.points = [];
        this.maxRange = 5.0;
        this.enabled = true;
        this.scale = 80;  // pixels per meter
        this.fadeAlpha = 0.6;

        // Colors based on distance
        this.colorNear = { r: 255, g: 50, b: 50 };    // Red = close
        this.colorMid = { r: 255, g: 200, b: 0 };     // Yellow = medium
        this.colorFar = { r: 0, g: 200, b: 255 };      // Cyan = far

        this.resize();
        window.addEventListener('resize', () => this.resize());
    }

    resize() {
        this.canvas.width = window.innerWidth;
        this.canvas.height = window.innerHeight;
        this.centerX = this.canvas.width / 2;
        this.centerY = this.canvas.height * 0.6;  // Robot position on screen
    }

    updateData(lidarData) {
        if (!this.enabled) return;
        this.points = lidarData.points || [];
        this.maxRange = lidarData.max_range || 5.0;
        this.render();
    }

    render() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        if (!this.enabled || this.points.length === 0) return;

        // Draw range rings
        this.drawRangeRings();

        // Draw LiDAR points
        for (const [x, y] of this.points) {
            const dist = Math.sqrt(x * x + y * y);
            const ratio = Math.min(1.0, dist / this.maxRange);

            // Screen coordinates (flip y for screen space)
            const sx = this.centerX + x * this.scale;
            const sy = this.centerY - y * this.scale;

            // Color based on distance
            const color = this.getDistanceColor(ratio);
            const size = Math.max(2, 6 - ratio * 4);

            this.ctx.beginPath();
            this.ctx.arc(sx, sy, size, 0, Math.PI * 2);
            this.ctx.fillStyle = `rgba(${color.r}, ${color.g}, ${color.b}, ${this.fadeAlpha})`;
            this.ctx.fill();

            // Glow effect for close points
            if (dist < 1.0) {
                this.ctx.beginPath();
                this.ctx.arc(sx, sy, size + 4, 0, Math.PI * 2);
                this.ctx.fillStyle = `rgba(255, 50, 50, 0.2)`;
                this.ctx.fill();
            }
        }

        // Draw robot indicator
        this.drawRobotIndicator();
    }

    drawRangeRings() {
        const rings = [1, 2, 3, 4, 5];

        for (const r of rings) {
            if (r > this.maxRange) break;
            
            const radius = r * this.scale;
            this.ctx.beginPath();
            this.ctx.arc(this.centerX, this.centerY, radius, 0, Math.PI * 2);
            this.ctx.strokeStyle = `rgba(0, 200, 255, 0.1)`;
            this.ctx.lineWidth = 1;
            this.ctx.stroke();

            // Label
            this.ctx.fillStyle = 'rgba(0, 200, 255, 0.3)';
            this.ctx.font = '10px monospace';
            this.ctx.fillText(`${r}m`, this.centerX + radius + 4, this.centerY - 4);
        }
    }

    drawRobotIndicator() {
        const size = 12;
        this.ctx.save();
        this.ctx.translate(this.centerX, this.centerY);

        // Robot triangle (pointing up = forward)
        this.ctx.beginPath();
        this.ctx.moveTo(0, -size);
        this.ctx.lineTo(-size * 0.7, size * 0.5);
        this.ctx.lineTo(size * 0.7, size * 0.5);
        this.ctx.closePath();

        this.ctx.fillStyle = 'rgba(0, 200, 255, 0.5)';
        this.ctx.fill();
        this.ctx.strokeStyle = 'rgba(0, 200, 255, 0.8)';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();

        this.ctx.restore();
    }

    getDistanceColor(ratio) {
        if (ratio < 0.3) {
            // Near: red to yellow
            const t = ratio / 0.3;
            return {
                r: Math.round(this.colorNear.r + (this.colorMid.r - this.colorNear.r) * t),
                g: Math.round(this.colorNear.g + (this.colorMid.g - this.colorNear.g) * t),
                b: Math.round(this.colorNear.b + (this.colorMid.b - this.colorNear.b) * t)
            };
        } else {
            // Mid to far: yellow to cyan
            const t = (ratio - 0.3) / 0.7;
            return {
                r: Math.round(this.colorMid.r + (this.colorFar.r - this.colorMid.r) * t),
                g: Math.round(this.colorMid.g + (this.colorFar.g - this.colorMid.g) * t),
                b: Math.round(this.colorMid.b + (this.colorFar.b - this.colorMid.b) * t)
            };
        }
    }

    toggle(enabled) {
        this.enabled = enabled;
        if (!enabled) {
            this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        }
    }
}
