/***********************
 * ROS TOPICS
 ***********************/

// Odometry (robot pose)
var odomTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/odom",
    messageType: "nav_msgs/msg/Odometry",
});

odomTopic.subscribe(function(message) {
    if (message.pose && message.pose.pose) {
        mapRenderer.updateRobotPose(
            message.pose.pose.position,
            message.pose.pose.orientation
        );
    }
});

var goalPoseTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/goal_pose",
    messageType: "geometry_msgs/PoseStamped",
});


/***********************
 * MAP RENDERER
 ***********************/

class MapRenderer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');

        this.canvas.width = 800;
        this.canvas.height = 600;

        this.mapData = null;

        this.zoom = 1.0;
        this.panX = 0;
        this.panY = 0;
        this.minZoom = 0.5;
        this.maxZoom = 5.0;

        this.robotPos = null;
        this.robotOri = null;

        this.goalPos = null;
        this.goalOrientation = 0; // Store goal orientation in radians

        this.isSettingGoal = false; // Flag for goal setting mode

        this.setupEventListeners();
        this.drawDefaultGrid();
    }

    updateRobotPose(pos, ori) {
        this.robotPos = pos;
        this.robotOri = ori;
        this.redraw();
    }

    setupEventListeners() {
        // Zoom
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const oldZoom = this.zoom;
            this.zoom += (e.deltaY > 0 ? -0.1 : 0.1);
            this.zoom = Math.max(this.minZoom, Math.min(this.maxZoom, this.zoom));

            const rect = this.canvas.getBoundingClientRect();
            const mx = e.clientX - rect.left;
            const my = e.clientY - rect.top;

            const ratio = this.zoom / oldZoom;
            this.panX = mx - (mx - this.panX) * ratio;
            this.panY = my - (my - this.panY) * ratio;

            document.getElementById('zoomLevel').textContent = this.zoom.toFixed(2) + 'x';
            this.redraw();
        });

        // Pan
        let dragging = false;
        let lastX = 0;
        let lastY = 0;

        this.canvas.addEventListener('mousedown', (e) => {
            if (this.isSettingGoal) return; // Don't pan when setting goal
            dragging = true;
            lastX = e.clientX;
            lastY = e.clientY;
        });

        document.addEventListener('mousemove', (e) => {
            if (!dragging) return;
            this.panX += e.clientX - lastX;
            this.panY += e.clientY - lastY;
            lastX = e.clientX;
            lastY = e.clientY;
            this.redraw();
        });

        document.addEventListener('mouseup', () => dragging = false);

        // CLICK TO SET GOAL
        this.canvas.addEventListener('click', (e) => {
            if (!this.mapData || !this.isSettingGoal) return;

            const rect = this.canvas.getBoundingClientRect();
            const px = e.clientX - rect.left;
            const py = e.clientY - rect.top;

            const world = this.screenToWorld(px, py);
            this.goalPos = world;
            
            // Show orientation selector
            this.showOrientationSelector(e.clientX, e.clientY);
        });

        // Update cursor based on mode
        this.canvas.addEventListener('mousemove', (e) => {
            if (this.isSettingGoal) {
                this.canvas.style.cursor = 'crosshair';
            } else {
                this.canvas.style.cursor = 'grab';
            }
        });
    }

    screenToWorld(px, py) {
        const info = this.mapData.info;
        const res = info.resolution;

        const x = (px - this.panX) / this.zoom;
        const y = (py - this.panY) / this.zoom;

        return {
            x: x * res + info.origin.position.x,
            y: -y * res - info.origin.position.y
        };
    }

    worldToScreen(wx, wy) {
        const info = this.mapData.info;
        const res = info.resolution;

        const x = (wx - info.origin.position.x) / res;
        const y = (-wy - info.origin.position.y) / res;

        return {
            x: x * this.zoom + this.panX,
            y: y * this.zoom + this.panY
        };
    }

    showOrientationSelector(clickX, clickY) {
        // Create a temporary overlay for orientation selection
        const overlay = document.createElement('div');
        overlay.style.position = 'fixed';
        overlay.style.left = '0';
        overlay.style.top = '0';
        overlay.style.width = '100%';
        overlay.style.height = '100%';
        overlay.style.background = 'rgba(0,0,0,0.3)';
        overlay.style.zIndex = '1000';
        overlay.style.cursor = 'crosshair';

        const instructions = document.createElement('div');
        instructions.style.position = 'fixed';
        instructions.style.left = '50%';
        instructions.style.top = '20px';
        instructions.style.transform = 'translateX(-50%)';
        instructions.style.background = '#333';
        instructions.style.color = '#fff';
        instructions.style.padding = '15px 30px';
        instructions.style.borderRadius = '8px';
        instructions.style.fontSize = '16px';
        instructions.style.boxShadow = '0 4px 6px rgba(0,0,0,0.3)';
        instructions.innerHTML = '🎯 Click to set goal direction (or press ESC to cancel)';

        overlay.appendChild(instructions);
        document.body.appendChild(overlay);

        const goalScreenPos = this.worldToScreen(this.goalPos.x, this.goalPos.y);

        overlay.addEventListener('click', (e) => {
            const dx = e.clientX - goalScreenPos.x;
            const dy = e.clientY - goalScreenPos.y;
            this.goalOrientation = Math.atan2(dy, dx);
            
            this.publishGoal();
            this.isSettingGoal = false;
            this.updateGoalButton();
            document.body.removeChild(overlay);
            this.redraw();
        });

        overlay.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') {
                this.goalPos = null;
                this.isSettingGoal = false;
                this.updateGoalButton();
                document.body.removeChild(overlay);
                this.redraw();
            }
        });

        overlay.focus();
    }

    publishGoal() {
        if (!this.goalPos) return;

        // Convert orientation to quaternion
        const qz = Math.sin(this.goalOrientation / 2);
        const qw = Math.cos(this.goalOrientation / 2);

        const msg = {
            header: { 
                frame_id: "map", 
                stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 } 
            },
            pose: {
                position: {
                    x: this.goalPos.x,
                    y: this.goalPos.y,
                    z: 0.0
                },
                orientation: {
                    x: 0.0, 
                    y: 0.0, 
                    z: qz, 
                    w: qw
                }
            }
        };

        console.log("Publishing goal:", msg);
        goalPoseTopic.publish(msg);
        
        this.showNotification('✅ Goal sent to robot!', 'success');
    }

    clearGoal() {
        this.goalPos = null;
        this.goalOrientation = 0;
        this.redraw();
        this.showNotification('🗑️ Goal cleared', 'info');
    }

    toggleGoalMode() {
        this.isSettingGoal = !this.isSettingGoal;
        this.updateGoalButton();
        
        if (this.isSettingGoal) {
            this.showNotification('🎯 Click on map to set goal position', 'info');
        } else {
            this.showNotification('Goal mode disabled', 'info');
        }
    }

    updateGoalButton() {
        const btn = document.getElementById('setGoalBtn');
        if (btn) {
            if (this.isSettingGoal) {
                btn.textContent = '❌ Cancel Goal';
                btn.style.background = '#dc3545';
            } else {
                btn.textContent = '🎯 Set Goal';
                btn.style.background = '#28a745';
            }
        }
    }

    showNotification(message, type = 'info') {
        const notification = document.createElement('div');
        notification.textContent = message;
        notification.style.position = 'fixed';
        notification.style.top = '80px';
        notification.style.right = '20px';
        notification.style.padding = '12px 20px';
        notification.style.borderRadius = '6px';
        notification.style.color = '#fff';
        notification.style.fontSize = '14px';
        notification.style.zIndex = '2000';
        notification.style.boxShadow = '0 4px 6px rgba(0,0,0,0.2)';
        notification.style.animation = 'slideIn 0.3s ease';
        
        if (type === 'success') {
            notification.style.background = '#28a745';
        } else if (type === 'error') {
            notification.style.background = '#dc3545';
        } else {
            notification.style.background = '#17a2b8';
        }

        document.body.appendChild(notification);

        setTimeout(() => {
            notification.style.animation = 'slideOut 0.3s ease';
            setTimeout(() => document.body.removeChild(notification), 300);
        }, 3000);
    }

    drawDefaultGrid() {
        this.ctx.fillStyle = "#1a1a1a";
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        
        // Draw grid lines
        this.ctx.strokeStyle = "#333";
        this.ctx.lineWidth = 1;
        
        for (let i = 0; i < this.canvas.width; i += 50) {
            this.ctx.beginPath();
            this.ctx.moveTo(i, 0);
            this.ctx.lineTo(i, this.canvas.height);
            this.ctx.stroke();
        }
        
        for (let i = 0; i < this.canvas.height; i += 50) {
            this.ctx.beginPath();
            this.ctx.moveTo(0, i);
            this.ctx.lineTo(this.canvas.width, i);
            this.ctx.stroke();
        }

        // Draw center crosshair
        this.ctx.strokeStyle = "#555";
        this.ctx.lineWidth = 2;
        const cx = this.canvas.width / 2;
        const cy = this.canvas.height / 2;
        
        this.ctx.beginPath();
        this.ctx.moveTo(cx - 20, cy);
        this.ctx.lineTo(cx + 20, cy);
        this.ctx.moveTo(cx, cy - 20);
        this.ctx.lineTo(cx, cy + 20);
        this.ctx.stroke();
    }

    updateMapData(map) {
        this.mapData = map;
        this.redraw();
        document.getElementById('mapStatus').textContent =
            `Map: ${map.info.width} x ${map.info.height} (${map.info.resolution.toFixed(3)}m/cell)`;
    }

    redraw() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        if (!this.mapData) {
            this.drawDefaultGrid();
            return;
        }
        this.renderMap();
    }

    renderMap() {
        const info = this.mapData.info;
        const data = this.mapData.data;
        const w = info.width;
        const h = info.height;

        this.ctx.save();
        this.ctx.translate(this.panX, this.panY);
        this.ctx.scale(this.zoom, this.zoom);

        // Draw map cells
        for (let i = 0; i < data.length; i++) {
            const v = data[i];
            if (v < 0) continue; // Unknown space
            
            // Gradient coloring based on occupancy
            if (v > 50) {
                this.ctx.fillStyle = "#000"; // Occupied
            } else {
                const gray = Math.floor(255 - (v * 2.55));
                this.ctx.fillStyle = `rgb(${gray}, ${gray}, ${gray})`;
            }
            
            const x = i % w;
            const y = Math.floor(i / w);
            this.ctx.fillRect(x, y, 1, 1);
        }

        // Draw robot
        if (this.robotPos) {
            const rx = (this.robotPos.x - info.origin.position.x) / info.resolution;
            const ry = (-this.robotPos.y - info.origin.position.y) / info.resolution;
            
            // Robot body
            this.ctx.fillStyle = "#0066FF";
            this.ctx.beginPath();
            this.ctx.arc(rx, ry, 5 / this.zoom, 0, Math.PI * 2);
            this.ctx.fill();
            
            // Robot direction indicator
            if (this.robotOri) {
                const yaw = this.quaternionToYaw(this.robotOri);
                this.ctx.strokeStyle = "#00CCFF";
                this.ctx.lineWidth = 2 / this.zoom;
                this.ctx.beginPath();
                this.ctx.moveTo(rx, ry);
                this.ctx.lineTo(
                    rx + Math.cos(yaw) * (10 / this.zoom),
                    ry + Math.sin(yaw) * (10 / this.zoom)
                );
                this.ctx.stroke();
            }
            
            // Robot label
            this.ctx.fillStyle = "#FFF";
            this.ctx.font = `${12 / this.zoom}px Arial`;
            this.ctx.fillText("🤖", rx + 8 / this.zoom, ry - 8 / this.zoom);
        }

        // Draw goal
        if (this.goalPos) {
            const gx = (this.goalPos.x - info.origin.position.x) / info.resolution;
            const gy = (-this.goalPos.y - info.origin.position.y) / info.resolution;
            
            // Goal outer ring
            this.ctx.strokeStyle = "#FF3333";
            this.ctx.lineWidth = 2 / this.zoom;
            this.ctx.beginPath();
            this.ctx.arc(gx, gy, 8 / this.zoom, 0, Math.PI * 2);
            this.ctx.stroke();
            
            // Goal center
            this.ctx.fillStyle = "#FF3333";
            this.ctx.beginPath();
            this.ctx.arc(gx, gy, 4 / this.zoom, 0, Math.PI * 2);
            this.ctx.fill();
            
            // Goal direction arrow
            this.ctx.strokeStyle = "#FF6666";
            this.ctx.lineWidth = 2 / this.zoom;
            this.ctx.beginPath();
            this.ctx.moveTo(gx, gy);
            const arrowLen = 15 / this.zoom;
            this.ctx.lineTo(
                gx + Math.cos(this.goalOrientation) * arrowLen,
                gy + Math.sin(this.goalOrientation) * arrowLen
            );
            this.ctx.stroke();
            
            // Arrow head
            const headSize = 5 / this.zoom;
            const angle = this.goalOrientation;
            this.ctx.beginPath();
            this.ctx.moveTo(
                gx + Math.cos(angle) * arrowLen,
                gy + Math.sin(angle) * arrowLen
            );
            this.ctx.lineTo(
                gx + Math.cos(angle) * arrowLen - Math.cos(angle - Math.PI / 6) * headSize,
                gy + Math.sin(angle) * arrowLen - Math.sin(angle - Math.PI / 6) * headSize
            );
            this.ctx.moveTo(
                gx + Math.cos(angle) * arrowLen,
                gy + Math.sin(angle) * arrowLen
            );
            this.ctx.lineTo(
                gx + Math.cos(angle) * arrowLen - Math.cos(angle + Math.PI / 6) * headSize,
                gy + Math.sin(angle) * arrowLen - Math.sin(angle + Math.PI / 6) * headSize
            );
            this.ctx.stroke();
            
            // Goal label
            this.ctx.fillStyle = "#FFF";
            this.ctx.font = `${12 / this.zoom}px Arial`;
            this.ctx.fillText("🎯", gx + 10 / this.zoom, gy - 10 / this.zoom);
        }

        this.ctx.restore();
    }

    quaternionToYaw(q) {
        // Convert quaternion to yaw angle
        return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }
}

/***********************
 * INIT
 ***********************/

const mapRenderer = new MapRenderer("mapCanvas");

// Map subscription
var mapTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/map",
    messageType: "nav_msgs/msg/OccupancyGrid",
});

mapTopic.subscribe(function(msg) {
    mapRenderer.updateMapData(msg);
});