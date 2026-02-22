// boids.js

// --- Configuration ---
const BOID_COUNT = 60;
// const MOBILE_BOID_COUNT = 40; // Fewer boids for mobile - REMOVED
const MAX_SPEED = 3;
const MAX_FORCE = 0.05;
const PERCEPTION_RADIUS = 50;
const SEPARATION_RADIUS = 25;
const ALIGNMENT_WEIGHT = 1.0;
const COHESION_WEIGHT = 1.0;
const SEPARATION_WEIGHT = 1.5;
const MOUSE_SEEK_WEIGHT = 1.0;
const EDGE_MARGIN = 50;
const TURN_FACTOR = 0.2; // How sharply boids turn from edges
const REPEL_RADIUS = 150; // Radius around click to repel boids
const REPEL_FORCE_WEIGHT = 2.0; // Strength of the repel force

// --- Vector Class ---
class Vector {
    constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }

    add(v) {
        this.x += v.x;
        this.y += v.y;
        return this;
    }

    sub(v) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }

    mult(s) {
        this.x *= s;
        this.y *= s;
        return this;
    }

    div(s) {
        if (s !== 0) {
            this.x /= s;
            this.y /= s;
        }
        return this;
    }

    magSq() {
        return this.x * this.x + this.y * this.y;
    }

    mag() {
        return Math.sqrt(this.magSq());
    }

    normalize() {
        const len = this.mag();
        if (len !== 0) {
            this.div(len);
        }
        return this;
    }

    setMag(magnitude) {
        this.normalize().mult(magnitude);
        return this;
    }

    limit(max) {
        const magSq = this.magSq();
        if (magSq > max * max) {
            this.div(Math.sqrt(magSq)).mult(max);
        }
        return this;
    }

    static sub(v1, v2) {
        return new Vector(v1.x - v2.x, v1.y - v2.y);
    }

    static distSq(v1, v2) {
        const dx = v1.x - v2.x;
        const dy = v1.y - v2.y;
        return dx * dx + dy * dy;
    }
}

// --- Boid Class ---
class Boid {
    constructor(canvasWidth, canvasHeight) {
        this.position = new Vector(Math.random() * canvasWidth, Math.random() * canvasHeight);
        this.velocity = new Vector(Math.random() * 2 - 1, Math.random() * 2 - 1);
        this.velocity.setMag(Math.random() * (MAX_SPEED - 1) + 1); // Random initial speed
        this.acceleration = new Vector();
        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;
    }

    // --- Boid Behavior ---
    align(boids) {
        let steering = new Vector();
        let total = 0;
        for (let other of boids) {
            const dSq = Vector.distSq(this.position, other.position);
            if (other !== this && dSq < PERCEPTION_RADIUS * PERCEPTION_RADIUS) {
                steering.add(other.velocity);
                total++;
            }
        }
        if (total > 0) {
            steering.div(total);
            steering.setMag(MAX_SPEED);
            steering.sub(this.velocity);
            steering.limit(MAX_FORCE);
        }
        return steering;
    }

    cohesion(boids) {
        let steering = new Vector();
        let total = 0;
        for (let other of boids) {
            const dSq = Vector.distSq(this.position, other.position);
            if (other !== this && dSq < PERCEPTION_RADIUS * PERCEPTION_RADIUS) {
                steering.add(other.position);
                total++;
            }
        }
        if (total > 0) {
            steering.div(total);
            steering.sub(this.position);
            steering.setMag(MAX_SPEED);
            steering.sub(this.velocity);
            steering.limit(MAX_FORCE);
        }
        return steering;
    }

    separation(boids) {
        let steering = new Vector();
        let total = 0;
        for (let other of boids) {
            const dSq = Vector.distSq(this.position, other.position);
            if (other !== this && dSq < SEPARATION_RADIUS * SEPARATION_RADIUS && dSq > 0) {
                let diff = Vector.sub(this.position, other.position);
                diff.div(dSq); // Weight by distance
                steering.add(diff);
                total++;
            }
        }
        if (total > 0) {
            steering.div(total);
            steering.setMag(MAX_SPEED);
            steering.sub(this.velocity);
            steering.limit(MAX_FORCE);
        }
        return steering;
    }

    seek(target) {
        if (!target) return new Vector();
        let desired = Vector.sub(target, this.position);
        // Check if distance is 0 to avoid division by zero if target is exactly at position
        if (desired.magSq() === 0) return new Vector();
        desired.setMag(MAX_SPEED);
        let steering = Vector.sub(desired, this.velocity);
        steering.limit(MAX_FORCE);
        return steering;
    }

    repel(target) {
        if (!target) return new Vector();
        let desired = Vector.sub(this.position, target); // Reversed direction from seek
        const dSq = desired.magSq();
        
        // Only apply force if within repel radius
        if (dSq > 0 && dSq < REPEL_RADIUS * REPEL_RADIUS) {
            desired.setMag(MAX_SPEED);
            let steering = Vector.sub(desired, this.velocity);
            steering.limit(MAX_FORCE); // Limit the force
            // Optionally, make repel stronger closer to the center
            // steering.mult(1 - Math.sqrt(dSq) / REPEL_RADIUS);
            return steering;
        }
        return new Vector();
    }

    avoidEdges() {
        const steer = new Vector();

        if (this.position.x < EDGE_MARGIN) {
            steer.x = MAX_SPEED;
        } else if (this.position.x > this.canvasWidth - EDGE_MARGIN) {
            steer.x = -MAX_SPEED;
        }

        if (this.position.y < EDGE_MARGIN) {
            steer.y = MAX_SPEED;
        } else if (this.position.y > this.canvasHeight - EDGE_MARGIN) {
            steer.y = -MAX_SPEED;
        }

        if (steer.x !== 0 || steer.y !== 0) {
            steer.setMag(MAX_SPEED);
            steer.sub(this.velocity);
            steer.limit(MAX_FORCE * 2); // Stronger avoidance force
        }

        return steer;
    }

    // Apply forces and update position/velocity
    flock(boids, mouseTarget, isRepellingActive) {
        let alignment = this.align(boids);
        let cohesion = this.cohesion(boids);
        let separation = this.separation(boids);
        let seekMouse = this.seek(mouseTarget);
        let avoid = this.avoidEdges();
        let repelForce = new Vector();

        // Apply repel force if mouse is down (isRepellingActive is true) and mouseTarget exists
        if (isRepellingActive && mouseTarget) {
            repelForce = this.repel(mouseTarget);
        }

        alignment.mult(ALIGNMENT_WEIGHT);
        cohesion.mult(COHESION_WEIGHT);
        separation.mult(SEPARATION_WEIGHT);
        seekMouse.mult(MOUSE_SEEK_WEIGHT);
        repelForce.mult(REPEL_FORCE_WEIGHT);

        this.acceleration.add(alignment);
        this.acceleration.add(cohesion);
        this.acceleration.add(separation);
        // Only seek if not repelling
        if (!isRepellingActive) {
            this.acceleration.add(seekMouse);
        }
        this.acceleration.add(avoid); // Add edge avoidance force
        this.acceleration.add(repelForce); // Add repel force (will be zero if not active)
    }

    update() {
        this.position.add(this.velocity);
        this.velocity.add(this.acceleration);
        this.velocity.limit(MAX_SPEED);
        this.acceleration.mult(0); // Reset acceleration each cycle
    }

    // Draw the boid as a triangle
    draw(ctx) {
        const angle = Math.atan2(this.velocity.y, this.velocity.x);
        ctx.save();
        ctx.translate(this.position.x, this.position.y);
        ctx.rotate(angle);
        ctx.beginPath();
        // Triangle shape
        ctx.moveTo(10, 0);
        ctx.lineTo(-5, -5);
        ctx.lineTo(-5, 5);
        ctx.closePath();

        // Determine color based on theme
        const isDarkMode = document.documentElement.getAttribute('data-theme') === 'dark';
        ctx.fillStyle = isDarkMode ? 'rgba(40, 60, 80, 0.3)' : 'rgba(173, 216, 230, 0.8)';
        ctx.fill();
        ctx.restore();
    }
}

// --- Simulation Setup ---
const canvas = document.createElement('canvas');
document.body.appendChild(canvas);
const ctx = canvas.getContext('2d');
let flock = [];
let mousePos = null; // Use null when mouse is out
let isRepelling = false; // Flag to indicate if mouse is held down
// let touchStartY = null; // REMOVED
// const SCROLL_THRESHOLD = 10; // REMOVED

function resizeCanvas() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    flock.forEach(boid => {
        boid.canvasWidth = canvas.width;
        boid.canvasHeight = canvas.height;
    });
}

// Helper function to update mousePos from event - REMOVED
/*
function updateTargetPosition(event) {
    // ... removed ...
}
*/

function setup() {
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    // Determine boid count based on screen size - REVERTED
    // const currentBoidCount = window.innerWidth < 768 ? MOBILE_BOID_COUNT : BOID_COUNT; // REMOVED

    // Initialize boids
    flock = [];
    for (let i = 0; i < BOID_COUNT; i++) { // Use BOID_COUNT directly
        flock.push(new Boid(canvas.width, canvas.height));
    }

    // --- Event Listeners (Reverted to mouse-only) ---

    // Mouse Movement
    document.addEventListener('mousemove', (event) => {
        // Simple update, no isRepelling check needed here in this version
        mousePos = new Vector(event.clientX, event.clientY);
    });
    document.addEventListener('mouseleave', () => {
        mousePos = null;
        isRepelling = false; // Stop repelling if mouse leaves while held down
    });

    // Mouse Click/Hold
    document.addEventListener('mousedown', (event) => {
        if (event.button === 0) { // Primary button
            isRepelling = true;
            mousePos = new Vector(event.clientX, event.clientY); // Update position immediately
        }
    });
    document.addEventListener('mouseup', (event) => {
        if (event.button === 0) { // Primary button
            isRepelling = false;
        }
    });

    // Touch Events - REMOVED
    /*
    const themeToggleButton = document.getElementById('theme-toggle');
    document.addEventListener('touchstart', ..., { passive: false });
    document.addEventListener('touchmove', ..., { passive: false });
    document.addEventListener('touchend', ...);
    document.addEventListener('touchcancel', ...);
    */

    // Start the animation loop
    animate();
}

function animate() {
    // Clear canvas each frame to avoid trails
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Update and draw each boid
    for (let boid of flock) {
        boid.flock(flock, mousePos, isRepelling); // Pass isRepelling flag
        boid.update();
        boid.draw(ctx);
    }

    requestAnimationFrame(animate);
}

// Add styles for the canvas
const style = document.createElement('style');
style.innerHTML = `
    canvas {
        position: fixed;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        z-index: -1; /* Place canvas behind other content */
        pointer-events: none; /* Allow interaction with elements behind canvas */
    }
`;
document.head.appendChild(style);


// Start the simulation when the DOM is ready
document.addEventListener('DOMContentLoaded', setup); 
