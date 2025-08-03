/**
 * Interactive 3D Underwater Vehicle Trajectory Planner
 * 
 * This system provides real-time visualization of different path planning algorithms:
 * - Dubins Curves: Smooth paths with minimum turning radius constraints
 * - RRT* (Rapidly-exploring Random Tree): Sampling-based planning for complex environments
 * - Bio-SALP: Bio-inspired jet propulsion planning mimicking salp/jellyfish movement
 */

class UnderwaterTrajectoryPlanner {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        
        // Planning state
        this.waypoints = [];
        this.obstacles = [];
        this.currentPath = null;
        this.vehicle = null;
        this.animationMixer = null;
        this.isAnimating = false;
        
        // Algorithm parameters
        this.currentAlgorithm = 'dubins';
        this.turnRadius = 1.5;
        this.animationSpeed = 1.0;
        
        // Interaction modes
        this.currentMode = 'VIEW'; // 'VIEW' or 'EDIT'
        this.selectedWaypoint = null;
        this.isDragging = false;
        this.dragOffset = new THREE.Vector3();
        
        // Visual elements
        this.pathLine = null;
        this.waypointMarkers = [];
        this.obstacleObjects = [];
        
        // Environment
        this.environment = 'open';
        this.waterSurface = null;
        this.seaFloor = null;
        
        this.init();
    }
    
    init() {
        this.setupScene();
        this.setupEnvironment();
        this.setupControls();
        this.setupEventListeners();
        this.animate();
        
        this.logMessage("🌊 Underwater Trajectory Planner initialized");
        this.logMessage("Click on the water surface to place waypoints");
    }
    
    setupScene() {
        // Create scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x001122);
        this.scene.fog = new THREE.Fog(0x001122, 10, 100);
        
        // Setup camera
        this.camera = new THREE.PerspectiveCamera(
            75, 
            window.innerWidth / window.innerHeight, 
            0.1, 
            1000
        );
        this.camera.position.set(15, 10, 15);
        this.camera.lookAt(0, 0, 0);
        
        // Setup renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        
        document.getElementById('canvas-container').appendChild(this.renderer.domElement);
        
        // Setup lighting
        this.setupLighting();
    }
    
    setupLighting() {
        // Ambient light (underwater ambience)
        const ambientLight = new THREE.AmbientLight(0x004080, 0.4);
        this.scene.add(ambientLight);
        
        // Directional light (filtered sunlight from surface)
        const directionalLight = new THREE.DirectionalLight(0x4080ff, 0.8);
        directionalLight.position.set(10, 20, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        directionalLight.shadow.camera.near = 0.5;
        directionalLight.shadow.camera.far = 50;
        directionalLight.shadow.camera.left = -20;
        directionalLight.shadow.camera.right = 20;
        directionalLight.shadow.camera.top = 20;
        directionalLight.shadow.camera.bottom = -20;
        this.scene.add(directionalLight);
        
        // Point lights for underwater atmosphere
        const pointLight1 = new THREE.PointLight(0x0080ff, 0.5, 30);
        pointLight1.position.set(-10, 5, -10);
        this.scene.add(pointLight1);
        
        const pointLight2 = new THREE.PointLight(0x00ff80, 0.3, 20);
        pointLight2.position.set(10, -5, 10);
        this.scene.add(pointLight2);
    }
    
    setupEnvironment() {
        this.createWaterSurface();
        this.createSeaFloor();
        this.createEnvironmentFeatures();
    }
    
    createWaterSurface() {
        // Animated water surface
        const waterGeometry = new THREE.PlaneGeometry(100, 100, 50, 50);
        const waterMaterial = new THREE.MeshPhongMaterial({
            color: 0x006699,
            transparent: true,
            opacity: 0.6,
            side: THREE.DoubleSide
        });
        
        this.waterSurface = new THREE.Mesh(waterGeometry, waterMaterial);
        this.waterSurface.rotation.x = -Math.PI / 2;
        this.waterSurface.position.y = 5;
        this.scene.add(this.waterSurface);
        
        // Add water animation
        this.animateWater();
    }
    
    animateWater() {
        const vertices = this.waterSurface.geometry.attributes.position.array;
        const time = Date.now() * 0.001;
        
        for (let i = 0; i < vertices.length; i += 3) {
            const x = vertices[i];
            const z = vertices[i + 2];
            vertices[i + 1] = Math.sin(x * 0.1 + time) * 0.2 + Math.cos(z * 0.1 + time * 0.7) * 0.1;
        }
        
        this.waterSurface.geometry.attributes.position.needsUpdate = true;
        this.waterSurface.geometry.computeVertexNormals();
    }
    
    createSeaFloor() {
        const floorGeometry = new THREE.PlaneGeometry(100, 100);
        const floorMaterial = new THREE.MeshLambertMaterial({ 
            color: 0x2d4a3e,
            transparent: true,
            opacity: 0.8
        });
        
        this.seaFloor = new THREE.Mesh(floorGeometry, floorMaterial);
        this.seaFloor.rotation.x = -Math.PI / 2;
        this.seaFloor.position.y = -15;
        this.seaFloor.receiveShadow = true;
        this.scene.add(this.seaFloor);
    }
    
    createEnvironmentFeatures() {
        // Add some basic underwater features
        this.addKelp();
        this.addRocks();
        this.addFish();
    }
    
    addKelp() {
        for (let i = 0; i < 10; i++) {
            const kelpGeometry = new THREE.CylinderGeometry(0.1, 0.2, 8, 8);
            const kelpMaterial = new THREE.MeshLambertMaterial({ color: 0x2d5a2d });
            const kelp = new THREE.Mesh(kelpGeometry, kelpMaterial);
            
            kelp.position.set(
                (Math.random() - 0.5) * 80,
                -11,
                (Math.random() - 0.5) * 80
            );
            kelp.castShadow = true;
            this.scene.add(kelp);
        }
    }
    
    addRocks() {
        for (let i = 0; i < 15; i++) {
            const rockGeometry = new THREE.SphereGeometry(
                Math.random() * 1.5 + 0.5, 
                8, 
                6
            );
            const rockMaterial = new THREE.MeshLambertMaterial({ color: 0x4a4a4a });
            const rock = new THREE.Mesh(rockGeometry, rockMaterial);
            
            rock.position.set(
                (Math.random() - 0.5) * 90,
                -14 + Math.random() * 2,
                (Math.random() - 0.5) * 90
            );
            rock.castShadow = true;
            rock.receiveShadow = true;
            this.scene.add(rock);
        }
    }
    
    addFish() {
        // Add some animated fish for atmosphere
        for (let i = 0; i < 8; i++) {
            const fishGeometry = new THREE.ConeGeometry(0.2, 0.8, 6);
            const fishMaterial = new THREE.MeshPhongMaterial({ 
                color: new THREE.Color().setHSL(Math.random(), 0.7, 0.6)
            });
            const fish = new THREE.Mesh(fishGeometry, fishMaterial);
            
            fish.position.set(
                (Math.random() - 0.5) * 60,
                Math.random() * 15 - 5,
                (Math.random() - 0.5) * 60
            );
            fish.rotation.z = Math.PI / 2;
            this.scene.add(fish);
            
            // Simple fish animation
            this.animateFish(fish);
        }
    }
    
    animateFish(fish) {
        const originalPosition = fish.position.clone();
        const radius = 5;
        const speed = 0.001 + Math.random() * 0.002;
        
        const animate = () => {
            const time = Date.now() * speed;
            fish.position.x = originalPosition.x + Math.cos(time) * radius;
            fish.position.z = originalPosition.z + Math.sin(time) * radius;
            fish.lookAt(
                fish.position.x + Math.cos(time + 0.1),
                fish.position.y,
                fish.position.z + Math.sin(time + 0.1)
            );
            requestAnimationFrame(animate);
        };
        animate();
    }
    
    setupControls() {
        // Mouse controls for camera
        this.controls = {
            mouseDown: false,
            mouseButton: 0,
            mouseX: 0,
            mouseY: 0,
            cameraDistance: 25,
            cameraAngleX: 0,
            cameraAngleY: 0
        };
        
        // Raycaster for clicking
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
    }
    
    setupEventListeners() {
        // Mouse events
        this.renderer.domElement.addEventListener('mousedown', (e) => this.onMouseDown(e));
        this.renderer.domElement.addEventListener('mousemove', (e) => this.onMouseMove(e));
        this.renderer.domElement.addEventListener('mouseup', (e) => this.onMouseUp(e));
        this.renderer.domElement.addEventListener('wheel', (e) => this.onWheel(e));
        this.renderer.domElement.addEventListener('click', (e) => this.onClick(e));
        
        // Keyboard events
        document.addEventListener('keydown', (e) => this.onKeyDown(e));
        
        // UI events
        this.setupUIEventListeners();
        
        // Window resize
        window.addEventListener('resize', () => this.onWindowResize());
    }
    
    setupUIEventListeners() {
        // Algorithm selection
        document.querySelectorAll('.algorithm-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                document.querySelectorAll('.algorithm-btn').forEach(b => b.classList.remove('active'));
                e.target.classList.add('active');
                this.currentAlgorithm = e.target.dataset.algorithm;
                this.logMessage(`🔄 Switched to ${this.getAlgorithmName(this.currentAlgorithm)} algorithm`);
                this.replanPath();
            });
        });
        
        // Turn radius slider
        const turnRadiusSlider = document.getElementById('turn-radius');
        const turnRadiusValue = document.getElementById('turn-radius-value');
        turnRadiusSlider.addEventListener('input', (e) => {
            this.turnRadius = parseFloat(e.target.value);
            turnRadiusValue.textContent = this.turnRadius.toFixed(1);
            this.replanPath();
        });
        
        // Animation speed slider
        const speedSlider = document.getElementById('animation-speed');
        const speedValue = document.getElementById('speed-value');
        speedSlider.addEventListener('input', (e) => {
            this.animationSpeed = parseFloat(e.target.value);
            speedValue.textContent = this.animationSpeed.toFixed(1);
        });
        
        // Environment selection
        document.getElementById('environment-select').addEventListener('change', (e) => {
            this.environment = e.target.value;
            this.updateEnvironment();
        });
        
        // Buttons
        document.getElementById('plan-path').addEventListener('click', () => this.planPath());
        document.getElementById('add-obstacle').addEventListener('click', () => this.addObstacle());
        document.getElementById('clear-scene').addEventListener('click', () => this.clearScene());
        document.getElementById('emergency-surface').addEventListener('click', () => this.emergencySurface());
        document.getElementById('animate-path').addEventListener('click', () => this.toggleAnimation());
    }
    
    onMouseDown(event) {
        this.controls.mouseDown = true;
        this.controls.mouseButton = event.button;
        this.controls.mouseX = event.clientX;
        this.controls.mouseY = event.clientY;
        
        // Check for waypoint dragging in EDIT mode
        if (this.currentMode === 'EDIT' && event.button === 0) {
            this.mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
            this.mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
            
            this.raycaster.setFromCamera(this.mouse, this.camera);
            const waypointIntersects = this.raycaster.intersectObjects(this.waypointMarkers);
            
            if (waypointIntersects.length > 0) {
                this.isDragging = true;
                this.selectedWaypoint = waypointIntersects[0].object;
                
                // Calculate drag offset
                const intersectionPoint = waypointIntersects[0].point;
                this.dragOffset.subVectors(this.selectedWaypoint.position, intersectionPoint);
                
                // Highlight the waypoint being dragged
                this.selectedWaypoint.material.emissive.setHex(0x666666);
                
                this.logMessage(`🖱️ Dragging waypoint ${this.waypointMarkers.indexOf(this.selectedWaypoint) + 1}`);
                return; // Don't start camera movement
            }
        }
    }
    
    onMouseMove(event) {
        if (!this.controls.mouseDown) return;
        
        // Handle waypoint dragging in EDIT mode
        if (this.currentMode === 'EDIT' && this.isDragging && this.selectedWaypoint) {
            this.mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
            this.mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
            
            // Cast ray to find new position
            this.raycaster.setFromCamera(this.mouse, this.camera);
            
            // Try intersecting with water surface first
            const waterIntersects = this.raycaster.intersectObject(this.waterSurface);
            let newPosition;
            
            if (waterIntersects.length > 0) {
                newPosition = waterIntersects[0].point;
                newPosition.y = Math.min(newPosition.y, 3); // Keep underwater
            } else {
                // Use virtual plane at depth -2
                const virtualPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 2);
                const ray = this.raycaster.ray;
                newPosition = new THREE.Vector3();
                ray.intersectPlane(virtualPlane, newPosition);
            }
            
            if (newPosition) {
                // Update waypoint position
                this.selectedWaypoint.position.copy(newPosition);
                
                // Update waypoints array
                const waypointIndex = this.waypointMarkers.indexOf(this.selectedWaypoint);
                if (waypointIndex !== -1) {
                    this.waypoints[waypointIndex].copy(newPosition);
                }
                
                // Replan path in real-time
                if (this.waypoints.length >= 2) {
                    this.replanPath();
                }
                
                this.updateDepthDisplay(newPosition.y);
            }
            return;
        }
        
        // Only allow camera movement in VIEW mode
        if (this.currentMode !== 'VIEW') return;
        
        const deltaX = event.clientX - this.controls.mouseX;
        const deltaY = event.clientY - this.controls.mouseY;
        
        if (this.controls.mouseButton === 0) { // Left button - rotate
            this.controls.cameraAngleY -= deltaX * 0.01;
            this.controls.cameraAngleX -= deltaY * 0.01;
            this.controls.cameraAngleX = Math.max(-Math.PI/2, Math.min(Math.PI/2, this.controls.cameraAngleX));
        } else if (this.controls.mouseButton === 2) { // Right button - pan
            const panSpeed = 0.05;
            this.camera.position.x += deltaX * panSpeed;
            this.camera.position.z += deltaY * panSpeed;
        }
        
        this.updateCameraPosition();
        this.controls.mouseX = event.clientX;
        this.controls.mouseY = event.clientY;
    }
    
    onMouseUp(event) {
        this.controls.mouseDown = false;
        
        // End waypoint dragging
        if (this.isDragging && this.selectedWaypoint) {
            this.isDragging = false;
            
            // Reset waypoint highlight
            const waypointIndex = this.waypointMarkers.indexOf(this.selectedWaypoint);
            this.selectedWaypoint.material.emissive.setHex(
                waypointIndex === 0 ? 0x002200 : 0x221100
            );
            
            this.logMessage(`✅ Waypoint ${waypointIndex + 1} moved to (${this.selectedWaypoint.position.x.toFixed(1)}, ${this.selectedWaypoint.position.y.toFixed(1)}, ${this.selectedWaypoint.position.z.toFixed(1)})`);
        }
    }
    
    onWheel(event) {
        event.preventDefault();
        
        // Only allow zoom in VIEW mode
        if (this.currentMode !== 'VIEW') return;
        
        this.controls.cameraDistance += event.deltaY * 0.01;
        this.controls.cameraDistance = Math.max(5, Math.min(100, this.controls.cameraDistance));
        this.updateCameraPosition();
    }
    
    onClick(event) {
        if (this.controls.mouseDown) return; // Ignore if dragging
        
        // Only allow waypoint placement in EDIT mode
        if (this.currentMode !== 'EDIT') return;
        
        // Calculate mouse position in normalized device coordinates
        this.mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
        this.mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
        
        // Check if clicking on existing waypoint for selection/deletion
        this.raycaster.setFromCamera(this.mouse, this.camera);
        const waypointIntersects = this.raycaster.intersectObjects(this.waypointMarkers);
        
        if (waypointIntersects.length > 0) {
            // Clicked on existing waypoint - select or delete it
            const clickedWaypoint = waypointIntersects[0].object;
            this.selectWaypoint(clickedWaypoint);
            return;
        }
        
        // Try to place waypoint at clicked location
        // First try intersecting with water surface
        const waterIntersects = this.raycaster.intersectObject(this.waterSurface);
        
        let targetPoint;
        if (waterIntersects.length > 0) {
            targetPoint = waterIntersects[0].point;
            targetPoint.y = Math.min(targetPoint.y, 3); // Keep underwater
            console.log('Water surface intersection at:', targetPoint);
        } else {
            // Create a virtual plane at depth -2 for more predictable placement
            const virtualPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 2);
            const ray = this.raycaster.ray;
            targetPoint = new THREE.Vector3();
            ray.intersectPlane(virtualPlane, targetPoint);
            console.log('Virtual plane intersection at:', targetPoint);
        }
        
        if (targetPoint) {
            this.addWaypoint(targetPoint);
        }
    }
    
    onKeyDown(event) {
        switch(event.code) {
            case 'Space':
                event.preventDefault();
                this.toggleAnimation();
                break;
            case 'KeyR':
                this.resetCamera();
                break;
            case 'KeyC':
                this.clearScene();
                break;
            case 'KeyE':
                event.preventDefault();
                this.toggleMode();
                break;
            case 'Delete':
            case 'Backspace':
                if (this.selectedWaypoint && this.currentMode === 'EDIT') {
                    this.deleteSelectedWaypoint();
                }
                break;
        }
    }
    
    selectWaypoint(waypointMesh) {
        // Deselect previous waypoint
        if (this.selectedWaypoint) {
            this.selectedWaypoint.material.emissive.setHex(
                this.waypointMarkers.indexOf(this.selectedWaypoint) === 0 ? 0x002200 : 0x221100
            );
        }
        
        // Select new waypoint
        this.selectedWaypoint = waypointMesh;
        this.selectedWaypoint.material.emissive.setHex(0x444444); // Highlight selected
        
        const waypointIndex = this.waypointMarkers.indexOf(waypointMesh) + 1;
        this.logMessage(`🎯 Waypoint ${waypointIndex} selected - Press Delete to remove`);
    }
    
    deleteSelectedWaypoint() {
        if (!this.selectedWaypoint) return;
        
        const waypointIndex = this.waypointMarkers.indexOf(this.selectedWaypoint);
        if (waypointIndex === -1) return;
        
        // Remove from scene
        this.scene.remove(this.selectedWaypoint);
        
        // Remove from arrays
        this.waypointMarkers.splice(waypointIndex, 1);
        this.waypoints.splice(waypointIndex, 1);
        
        this.logMessage(`🗑️ Waypoint ${waypointIndex + 1} deleted`);
        
        // Clear selection
        this.selectedWaypoint = null;
        
        // Update waypoint labels
        this.updateWaypointLabels();
        
        // Replan path if we still have enough waypoints
        if (this.waypoints.length >= 2) {
            this.replanPath();
        } else {
            this.clearPath();
            this.updateMetrics(0, 0, 0);
        }
    }
    
    updateWaypointLabels() {
        // Update all waypoint labels and colors after deletion
        this.waypointMarkers.forEach((marker, index) => {
            // Update color
            marker.material.color.setHex(index === 0 ? 0x00ff00 : 0xff6600);
            marker.material.emissive.setHex(index === 0 ? 0x002200 : 0x221100);
            
            // Update label
            const sprite = marker.children.find(child => child.type === 'Sprite');
            if (sprite) {
                const canvas = document.createElement('canvas');
                const context = canvas.getContext('2d');
                canvas.width = 64;
                canvas.height = 64;
                
                context.fillStyle = 'rgba(0, 0, 0, 0.8)';
                context.fillRect(0, 0, 64, 64);
                context.fillStyle = 'white';
                context.font = 'bold 32px Arial';
                context.textAlign = 'center';
                context.fillText((index + 1).toString(), 32, 42);
                
                const texture = new THREE.CanvasTexture(canvas);
                sprite.material.map = texture;
                sprite.material.needsUpdate = true;
            }
        });
    }
    
    toggleMode() {
        this.currentMode = this.currentMode === 'VIEW' ? 'EDIT' : 'VIEW';
        document.getElementById('current-mode').textContent = this.currentMode;
        
        if (this.currentMode === 'EDIT') {
            this.logMessage("✏️ Switched to EDIT mode - click water to place waypoints");
        } else {
            this.logMessage("👁️ Switched to VIEW mode - drag to move camera");
        }
    }
    
    updateCameraPosition() {
        const x = this.controls.cameraDistance * Math.cos(this.controls.cameraAngleX) * Math.cos(this.controls.cameraAngleY);
        const y = this.controls.cameraDistance * Math.sin(this.controls.cameraAngleX);
        const z = this.controls.cameraDistance * Math.cos(this.controls.cameraAngleX) * Math.sin(this.controls.cameraAngleY);
        
        this.camera.position.set(x, y, z);
        this.camera.lookAt(0, 0, 0);
    }
    
    resetCamera() {
        this.controls.cameraDistance = 25;
        this.controls.cameraAngleX = 0.3;
        this.controls.cameraAngleY = 0.5;
        this.updateCameraPosition();
        this.logMessage("📷 Camera view reset");
    }
    
    addWaypoint(position) {
        // Clamp to underwater (below surface)
        position.y = Math.min(position.y, 3);
        
        this.waypoints.push(position.clone());
        
        // Create visual marker
        const markerGeometry = new THREE.SphereGeometry(0.3, 16, 16);
        const markerMaterial = new THREE.MeshPhongMaterial({ 
            color: this.waypoints.length === 1 ? 0x00ff00 : 0xff6600,
            emissive: this.waypoints.length === 1 ? 0x002200 : 0x221100
        });
        const marker = new THREE.Mesh(markerGeometry, markerMaterial);
        marker.position.copy(position);
        marker.castShadow = true;
        this.scene.add(marker);
        this.waypointMarkers.push(marker);
        
        // Add label
        this.addWaypointLabel(marker, this.waypoints.length);
        
        this.logMessage(`📍 Waypoint ${this.waypoints.length} added at (${position.x.toFixed(1)}, ${position.y.toFixed(1)}, ${position.z.toFixed(1)})`);
        
        // Auto-plan if we have at least 2 waypoints
        if (this.waypoints.length >= 2) {
            this.planPath();
        }
        
        this.updateDepthDisplay(position.y);
    }
    
    addWaypointLabel(marker, number) {
        // Create text sprite for waypoint number
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        canvas.width = 64;
        canvas.height = 64;
        
        context.fillStyle = 'rgba(0, 0, 0, 0.8)';
        context.fillRect(0, 0, 64, 64);
        context.fillStyle = 'white';
        context.font = 'bold 32px Arial';
        context.textAlign = 'center';
        context.fillText(number.toString(), 32, 42);
        
        const texture = new THREE.CanvasTexture(canvas);
        const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
        const sprite = new THREE.Sprite(spriteMaterial);
        sprite.scale.set(1, 1, 1);
        sprite.position.set(0, 1, 0);
        marker.add(sprite);
    }
    
    addObstacle() {
        // Add obstacle at random position
        const position = new THREE.Vector3(
            (Math.random() - 0.5) * 20,
            Math.random() * 8 - 2,
            (Math.random() - 0.5) * 20
        );
        
        const radius = Math.random() * 2 + 1;
        
        const obstacleGeometry = new THREE.SphereGeometry(radius, 16, 16);
        const obstacleMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xff4444,
            transparent: true,
            opacity: 0.7
        });
        const obstacle = new THREE.Mesh(obstacleGeometry, obstacleMaterial);
        obstacle.position.copy(position);
        obstacle.castShadow = true;
        this.scene.add(obstacle);
        
        this.obstacles.push({ position, radius, mesh: obstacle });
        this.obstacleObjects.push(obstacle);
        
        this.logMessage(`🚫 Obstacle added at (${position.x.toFixed(1)}, ${position.y.toFixed(1)}, ${position.z.toFixed(1)})`);
        
        // Replan path if we have waypoints
        if (this.waypoints.length >= 2) {
            this.replanPath();
        }
    }
    
    planPath() {
        if (this.waypoints.length < 2) {
            this.logMessage("⚠️ Need at least 2 waypoints to plan path");
            return;
        }
        
        this.setStatus('planning', 'Planning trajectory...');
        
        const startTime = performance.now();
        
        // Clear existing path
        this.clearPath();
        
        // Plan path based on selected algorithm
        let pathPoints;
        switch(this.currentAlgorithm) {
            case 'dubins':
                pathPoints = this.planDubinsPath();
                break;
            case 'bio':
                pathPoints = this.planBioInspiredPath();
                break;
            case 'rrt':
                pathPoints = this.planRRTPath();
                break;
            default:
                pathPoints = this.planDubinsPath();
        }
        
        const planningTime = performance.now() - startTime;
        
        if (pathPoints && pathPoints.length > 0) {
            this.createPathVisualization(pathPoints);
            this.currentPath = pathPoints;
            
            const pathLength = this.calculatePathLength(pathPoints);
            const energyCost = this.calculateEnergyCost(pathPoints);
            
            this.updateMetrics(pathLength, energyCost, planningTime);
            this.setStatus('ready', `Path planned successfully (${this.getAlgorithmName(this.currentAlgorithm)})`);
            
            this.logMessage(`✅ ${this.getAlgorithmName(this.currentAlgorithm)} path planned: ${pathLength.toFixed(1)}m, ${planningTime.toFixed(1)}ms`);
        } else {
            this.setStatus('error', 'Path planning failed');
            this.logMessage("❌ Path planning failed - no feasible path found");
        }
    }
    
    planDubinsPath() {
        // Simplified Dubins path implementation for visualization
        const pathPoints = [];
        
        for (let i = 0; i < this.waypoints.length - 1; i++) {
            const start = this.waypoints[i];
            const end = this.waypoints[i + 1];
            
            // Calculate direction vectors
            const direction = new THREE.Vector3().subVectors(end, start).normalize();
            const distance = start.distanceTo(end);
            
            // Create smooth curve with turn radius constraint
            const segmentPoints = this.createDubinsSegment(start, end, direction, this.turnRadius);
            pathPoints.push(...segmentPoints);
        }
        
        return pathPoints;
    }
    
    createDubinsSegment(start, end, direction, turnRadius) {
        const points = [];
        const numPoints = 20;
        
        // Simple implementation: create a smooth curve
        for (let i = 0; i <= numPoints; i++) {
            const t = i / numPoints;
            
            // Linear interpolation with smooth curves at turns
            const point = new THREE.Vector3().lerpVectors(start, end, t);
            
            // Add some curvature based on turn radius
            if (t > 0.1 && t < 0.9) {
                const curvature = Math.sin(t * Math.PI) * 0.5;
                point.y += curvature;
            }
            
            points.push(point);
        }
        
        return points;
    }
    
    planBioInspiredPath() {
        // Bio-inspired SALP/jellyfish movement with pulsing motion
        const pathPoints = [];
        
        for (let i = 0; i < this.waypoints.length - 1; i++) {
            const start = this.waypoints[i];
            const end = this.waypoints[i + 1];
            
            // Create pulsing motion segments (like jellyfish propulsion)
            const segmentPoints = this.createBioInspiredSegment(start, end);
            pathPoints.push(...segmentPoints);
        }
        
        return pathPoints;
    }
    
    createBioInspiredSegment(start, end) {
        const points = [];
        const numPulses = 8; // Number of jet propulsion events
        const pulsesPerSegment = 4;
        
        for (let pulse = 0; pulse < numPulses; pulse++) {
            const pulseProgress = pulse / (numPulses - 1);
            
            // Base position along straight line
            const basePoint = new THREE.Vector3().lerpVectors(start, end, pulseProgress);
            
            // Add pulsing motion (perpendicular to direction)
            const direction = new THREE.Vector3().subVectors(end, start).normalize();
            const perpendicular = new THREE.Vector3(-direction.z, 0, direction.x).normalize();
            
            // Create pulse points
            for (let p = 0; p < pulsesPerSegment; p++) {
                const pulsePhase = (p / pulsesPerSegment) * Math.PI * 2;
                const pulseAmplitude = 0.5 * Math.sin(pulse * Math.PI / numPulses);
                
                const point = basePoint.clone();
                point.add(perpendicular.clone().multiplyScalar(Math.sin(pulsePhase) * pulseAmplitude));
                point.y += Math.cos(pulsePhase) * pulseAmplitude * 0.3;
                
                points.push(point);
            }
        }
        
        return points;
    }
    
    planRRTPath() {
        // Simplified RRT* implementation for visualization
        const pathPoints = [];
        const maxIterations = 100;
        const stepSize = 2.0;
        
        // RRT tree structure
        const tree = [{ point: this.waypoints[0].clone(), parent: null }];
        const goal = this.waypoints[this.waypoints.length - 1];
        let goalReached = false;
        
        for (let i = 0; i < maxIterations && !goalReached; i++) {
            // Sample random point
            let randomPoint;
            if (Math.random() < 0.1) { // 10% chance to sample goal
                randomPoint = goal.clone();
            } else {
                randomPoint = new THREE.Vector3(
                    (Math.random() - 0.5) * 40,
                    Math.random() * 10 - 2,
                    (Math.random() - 0.5) * 40
                );
            }
            
            // Find nearest node in tree
            let nearestNode = tree[0];
            let nearestDistance = nearestNode.point.distanceTo(randomPoint);
            
            for (const node of tree) {
                const distance = node.point.distanceTo(randomPoint);
                if (distance < nearestDistance) {
                    nearestNode = node;
                    nearestDistance = distance;
                }
            }
            
            // Create new point towards random point
            const direction = new THREE.Vector3().subVectors(randomPoint, nearestNode.point).normalize();
            const newPoint = nearestNode.point.clone().add(direction.multiplyScalar(stepSize));
            
            // Check for collisions with obstacles
            if (!this.checkCollision(nearestNode.point, newPoint)) {
                const newNode = { point: newPoint, parent: nearestNode };
                tree.push(newNode);
                
                // Check if we reached the goal
                if (newPoint.distanceTo(goal) < stepSize) {
                    tree.push({ point: goal.clone(), parent: newNode });
                    goalReached = true;
                }
            }
        }
        
        if (goalReached) {
            // Reconstruct path from goal to start
            const path = [];
            let currentNode = tree[tree.length - 1]; // Goal node
            
            while (currentNode) {
                path.unshift(currentNode.point);
                currentNode = currentNode.parent;
            }
            
            // Smooth the path
            return this.smoothPath(path);
        }
        
        return null; // No path found
    }
    
    checkCollision(start, end) {
        // Check if line segment intersects with any obstacles
        for (const obstacle of this.obstacles) {
            const distance = this.distanceToLineSegment(start, end, obstacle.position);
            if (distance < obstacle.radius + 0.5) { // Safety margin
                return true;
            }
        }
        return false;
    }
    
    distanceToLineSegment(start, end, point) {
        const line = new THREE.Vector3().subVectors(end, start);
        const lineLength = line.length();
        
        if (lineLength === 0) return start.distanceTo(point);
        
        const t = Math.max(0, Math.min(1, new THREE.Vector3().subVectors(point, start).dot(line) / (lineLength * lineLength)));
        const projection = start.clone().add(line.multiplyScalar(t));
        
        return point.distanceTo(projection);
    }
    
    smoothPath(path) {
        if (path.length < 3) return path;
        
        const smoothed = [path[0]];
        
        for (let i = 1; i < path.length - 1; i++) {
            const prev = path[i - 1];
            const curr = path[i];
            const next = path[i + 1];
            
            // Simple smoothing: average with neighbors
            const smoothPoint = new THREE.Vector3()
                .addVectors(prev, curr)
                .add(next)
                .divideScalar(3);
            
            smoothed.push(smoothPoint);
        }
        
        smoothed.push(path[path.length - 1]);
        return smoothed;
    }
    
    createPathVisualization(pathPoints) {
        if (pathPoints.length < 2) return;
        
        // Create path geometry
        const pathGeometry = new THREE.BufferGeometry().setFromPoints(pathPoints);
        
        // Create path material based on algorithm
        let pathMaterial;
        switch(this.currentAlgorithm) {
            case 'dubins':
                pathMaterial = new THREE.LineBasicMaterial({ 
                    color: 0x00ff88,
                    linewidth: 3
                });
                break;
            case 'bio':
                pathMaterial = new THREE.LineBasicMaterial({ 
                    color: 0xff6600,
                    linewidth: 3
                });
                break;
            case 'rrt':
                pathMaterial = new THREE.LineBasicMaterial({ 
                    color: 0x6600ff,
                    linewidth: 3
                });
                break;
            default:
                pathMaterial = new THREE.LineBasicMaterial({ 
                    color: 0x00ff88,
                    linewidth: 3
                });
        }
        
        // Create path line
        this.pathLine = new THREE.Line(pathGeometry, pathMaterial);
        this.scene.add(this.pathLine);
        
        // Add path direction indicators
        this.addPathDirectionIndicators(pathPoints);
    }
    
    addPathDirectionIndicators(pathPoints) {
        // Add arrows along the path to show direction
        const arrowGeometry = new THREE.ConeGeometry(0.2, 0.6, 6);
        const arrowMaterial = new THREE.MeshPhongMaterial({ color: 0xffffff });
        
        for (let i = 5; i < pathPoints.length - 5; i += 10) {
            const point = pathPoints[i];
            const nextPoint = pathPoints[i + 1];
            
            const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
            arrow.position.copy(point);
            
            // Point arrow in direction of travel
            const direction = new THREE.Vector3().subVectors(nextPoint, point).normalize();
            arrow.lookAt(point.clone().add(direction));
            arrow.rotateX(Math.PI / 2);
            
            this.scene.add(arrow);
        }
    }
    
    clearPath() {
        if (this.pathLine) {
            this.scene.remove(this.pathLine);
            this.pathLine = null;
        }
        
        // Remove direction indicators
        const objectsToRemove = [];
        this.scene.traverse((child) => {
            if (child.geometry && child.geometry.type === 'ConeGeometry' && child.material.color.getHex() === 0xffffff) {
                objectsToRemove.push(child);
            }
        });
        objectsToRemove.forEach(obj => this.scene.remove(obj));
    }
    
    replanPath() {
        if (this.waypoints.length >= 2) {
            this.planPath();
        }
    }
    
    calculatePathLength(pathPoints) {
        let length = 0;
        for (let i = 1; i < pathPoints.length; i++) {
            length += pathPoints[i].distanceTo(pathPoints[i - 1]);
        }
        return length;
    }
    
    calculateEnergyCost(pathPoints) {
        // Simple energy cost calculation based on path characteristics
        const pathLength = this.calculatePathLength(pathPoints);
        let energyCost = pathLength;
        
        // Add cost for turns and depth changes
        for (let i = 1; i < pathPoints.length - 1; i++) {
            const prev = pathPoints[i - 1];
            const curr = pathPoints[i];
            const next = pathPoints[i + 1];
            
            // Calculate turn angle
            const v1 = new THREE.Vector3().subVectors(curr, prev).normalize();
            const v2 = new THREE.Vector3().subVectors(next, curr).normalize();
            const turnAngle = Math.acos(Math.max(-1, Math.min(1, v1.dot(v2))));
            
            // Add energy cost for turns
            energyCost += turnAngle * 2;
            
            // Add cost for depth changes
            const depthChange = Math.abs(next.y - curr.y);
            energyCost += depthChange * 1.5;
        }
        
        // Algorithm-specific energy modifiers
        switch(this.currentAlgorithm) {
            case 'bio':
                energyCost *= 1.2; // Bio-inspired uses more energy for pulsing
                break;
            case 'rrt':
                energyCost *= 0.9; // RRT* finds more efficient paths
                break;
        }
        
        return energyCost;
    }
    
    updateMetrics(pathLength, energyCost, planningTime) {
        document.getElementById('path-length').textContent = pathLength.toFixed(1);
        document.getElementById('energy-cost').textContent = energyCost.toFixed(1);
        document.getElementById('planning-time').textContent = planningTime.toFixed(1);
    }
    
    updateDepthDisplay(depth) {
        document.getElementById('depth').textContent = Math.abs(depth).toFixed(1);
    }
    
    setStatus(status, message) {
        const statusIndicator = document.querySelector('.status-indicator');
        const statusText = document.getElementById('status-text');
        
        statusIndicator.className = `status-indicator status-${status}`;
        statusText.textContent = message;
    }
    
    logMessage(message) {
        const log = document.getElementById('mission-log');
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.textContent = `[${timestamp}] ${message}`;
        log.appendChild(logEntry);
        log.scrollTop = log.scrollHeight;
    }
    
    getAlgorithmName(algorithm) {
        switch(algorithm) {
            case 'dubins': return 'Dubins Curves';
            case 'bio': return 'Bio-SALP';
            case 'rrt': return 'RRT*';
            default: return 'Unknown';
        }
    }
    
    clearScene() {
        // Clear waypoints
        this.waypointMarkers.forEach(marker => this.scene.remove(marker));
        this.waypointMarkers = [];
        this.waypoints = [];
        
        // Clear obstacles
        this.obstacleObjects.forEach(obstacle => this.scene.remove(obstacle));
        this.obstacleObjects = [];
        this.obstacles = [];
        
        // Clear path
        this.clearPath();
        this.currentPath = null;
        
        // Clear vehicle
        if (this.vehicle) {
            this.scene.remove(this.vehicle);
            this.vehicle = null;
        }
        
        // Reset metrics
        this.updateMetrics(0, 0, 0);
        this.updateDepthDisplay(0);
        this.setStatus('ready', 'Scene cleared - ready for new mission');
        this.logMessage("🗑️ Scene cleared");
    }
    
    emergencySurface() {
        if (!this.currentPath || this.currentPath.length === 0) {
            this.logMessage("⚠️ No active path for emergency surface");
            return;
        }
        
        // Create emergency surface path from current position to surface
        const currentPos = this.currentPath[0].clone();
        const surfacePos = new THREE.Vector3(currentPos.x, 4, currentPos.z);
        
        const emergencyPath = [currentPos, surfacePos];
        this.clearPath();
        this.createPathVisualization(emergencyPath);
        this.currentPath = emergencyPath;
        
        this.logMessage("🚨 Emergency surface procedure initiated");
        this.setStatus('planning', 'Emergency surface in progress');
    }
    
    toggleAnimation() {
        if (!this.currentPath || this.currentPath.length === 0) {
            this.logMessage("⚠️ No path to animate");
            return;
        }
        
        if (this.isAnimating) {
            this.stopAnimation();
        } else {
            this.startAnimation();
        }
    }
    
    startAnimation() {
        if (!this.currentPath) return;
        
        this.isAnimating = true;
        document.getElementById('animate-path').textContent = '⏸️ Stop Animation';
        
        // Create vehicle if it doesn't exist
        if (!this.vehicle) {
            this.createVehicle();
        }
        
        // Start animation along path
        this.animateVehicleAlongPath();
        this.logMessage("▶️ Vehicle animation started");
    }
    
    stopAnimation() {
        this.isAnimating = false;
        document.getElementById('animate-path').textContent = '▶️ Animate Vehicle';
        this.logMessage("⏸️ Vehicle animation stopped");
    }
    
    createVehicle() {
        // Create a simple underwater vehicle model
        const vehicleGroup = new THREE.Group();
        
        // Main body (torpedo shape)
        const bodyGeometry = new THREE.CylinderGeometry(0.3, 0.2, 2, 12);
        const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x4444ff });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.rotation.z = Math.PI / 2;
        vehicleGroup.add(body);
        
        // Fins
        const finGeometry = new THREE.BoxGeometry(0.1, 0.5, 0.8);
        const finMaterial = new THREE.MeshPhongMaterial({ color: 0x2222aa });
        
        const topFin = new THREE.Mesh(finGeometry, finMaterial);
        topFin.position.set(-0.5, 0.4, 0);
        vehicleGroup.add(topFin);
        
        const bottomFin = new THREE.Mesh(finGeometry, finMaterial);
        bottomFin.position.set(-0.5, -0.4, 0);
        vehicleGroup.add(bottomFin);
        
        // Propeller
        const propGeometry = new THREE.CylinderGeometry(0.4, 0.4, 0.1, 6);
        const propMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 });
        const propeller = new THREE.Mesh(propGeometry, propMaterial);
        propeller.position.set(-1.2, 0, 0);
        propeller.rotation.z = Math.PI / 2;
        vehicleGroup.add(propeller);
        
        // Lights
        const lightGeometry = new THREE.SphereGeometry(0.1, 8, 8);
        const lightMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xffff00,
            emissive: 0x444400
        });
        const light = new THREE.Mesh(lightGeometry, lightMaterial);
        light.position.set(1, 0, 0);
        vehicleGroup.add(light);
        
        this.vehicle = vehicleGroup;
        this.vehicle.position.copy(this.currentPath[0]);
        this.scene.add(this.vehicle);
    }
    
    animateVehicleAlongPath() {
        if (!this.isAnimating || !this.vehicle || !this.currentPath) return;
        
        const pathLength = this.currentPath.length;
        let currentIndex = 0;
        const speed = this.animationSpeed * 0.02;
        
        const animate = () => {
            if (!this.isAnimating) return;
            
            currentIndex += speed;
            
            if (currentIndex >= pathLength - 1) {
                currentIndex = 0; // Loop animation
            }
            
            const index = Math.floor(currentIndex);
            const t = currentIndex - index;
            
            if (index < pathLength - 1) {
                // Interpolate position
                const currentPos = this.currentPath[index];
                const nextPos = this.currentPath[index + 1];
                const interpolatedPos = new THREE.Vector3().lerpVectors(currentPos, nextPos, t);
                
                this.vehicle.position.copy(interpolatedPos);
                
                // Orient vehicle towards movement direction
                const direction = new THREE.Vector3().subVectors(nextPos, currentPos).normalize();
                this.vehicle.lookAt(this.vehicle.position.clone().add(direction));
                
                // Update depth display
                this.updateDepthDisplay(interpolatedPos.y);
            }
            
            requestAnimationFrame(animate);
        };
        
        animate();
    }
    
    updateEnvironment() {
        // Update environment based on selection
        switch(this.environment) {
            case 'coral':
                this.seaFloor.material.color.setHex(0x4a3a2a);
                this.logMessage("🪸 Environment changed to Coral Reef");
                break;
            case 'kelp':
                this.seaFloor.material.color.setHex(0x2a4a2a);
                this.logMessage("🌿 Environment changed to Kelp Forest");
                break;
            case 'thermal':
                this.seaFloor.material.color.setHex(0x4a2a2a);
                this.logMessage("🌋 Environment changed to Thermal Vents");
                break;
            default:
                this.seaFloor.material.color.setHex(0x2d4a3e);
                this.logMessage("🌊 Environment changed to Open Water");
        }
    }
    
    onWindowResize() {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    }
    
    animate() {
        requestAnimationFrame(() => this.animate());
        
        // Animate water surface
        this.animateWater();
        
        // Render scene
        this.renderer.render(this.scene, this.camera);
    }
}

// Initialize the planner when the page loads
document.addEventListener('DOMContentLoaded', () => {
    new UnderwaterTrajectoryPlanner();
});
