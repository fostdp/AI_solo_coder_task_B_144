/* ============================================================
 * chariot_3d.js - 双辕车三维渲染模块
 * 职责：Three.js场景、3D模型构建、GPU实例化、动画渲染
 * 依赖：three.min.js
 * 对外接口：Chariot3D 类
 * ============================================================ */

const CHARIOT_WHEELBASE = 2.5;
const CHARIOT_TRACK_WIDTH = 1.8;
const WHEEL_RADIUS = 0.35;
const POLE_LENGTH = 1.8;
const MAX_POINTS = 400;
const NUM_SPOKES = 8;

class Chariot3D {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.chariotGroup = null;
        this.wheelGroups = {};
        this.leftKingpin = null;
        this.rightKingpin = null;
        this.poleGroup = null;
        this.trajectoryLine = null;
        this.innerTrajectoryLine = null;
        this.trajectoryPoints = [];
        this.innerTrajectoryPoints = [];
        this.clock = new THREE.Clock();
        this.lastTime = performance.now();
        this.currentPoleAngle = 0;
        this.currentVehicleSpeed = 0;

        this._geometryCache = new Map();
        this._materialCache = new Map();
        this._spokeMeshes = [];

        this.init();
    }

    getSharedGeometry(key, factory) {
        if (!this._geometryCache.has(key)) {
            this._geometryCache.set(key, factory());
        }
        return this._geometryCache.get(key);
    }

    getSharedMaterial(key, factory) {
        if (!this._materialCache.has(key)) {
            this._materialCache.set(key, factory());
        }
        return this._materialCache.get(key);
    }

    createInstancedMesh(geometry, material, count, setupCallback) {
        const instanced = new THREE.InstancedMesh(geometry, material, count);
        instanced.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        const dummy = new THREE.Object3D();
        for (let i = 0; i < count; i++) {
            if (setupCallback) setupCallback(i, dummy);
            dummy.updateMatrix();
            instanced.setMatrixAt(i, dummy.matrix);
        }
        instanced.instanceMatrix.needsUpdate = true;
        return instanced;
    }

    init() {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x87ceeb);

        const width = this.container.clientWidth;
        const height = this.container.clientHeight;

        this.camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 1000);
        this.camera.position.set(5, 4, 6);
        this.camera.lookAt(0, 0.5, 0);

        this.renderer = new THREE.WebGLRenderer({
            antialias: true,
            powerPreference: "high-performance"
        });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.container.appendChild(this.renderer.domElement);

        this._setupLights();
        this._setupGround();
        this._buildChariot();
        this._setupTrajectory();

        window.addEventListener('resize', () => this._onResize());

        this.animate();
    }

    _setupLights() {
        const ambient = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambient);

        const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(10, 15, 8);
        dirLight.castShadow = true;
        dirLight.shadow.mapSize.width = 2048;
        dirLight.shadow.mapSize.height = 2048;
        dirLight.shadow.camera.near = 0.5;
        dirLight.shadow.camera.far = 50;
        dirLight.shadow.camera.left = -20;
        dirLight.shadow.camera.right = 20;
        dirLight.shadow.camera.top = 20;
        dirLight.shadow.camera.bottom = -20;
        this.scene.add(dirLight);

        const fillLight = new THREE.PointLight(0xffd700, 0.4, 20);
        fillLight.position.set(-5, 5, -5);
        this.scene.add(fillLight);
    }

    _setupGround() {
        const gridHelper = new THREE.GridHelper(50, 50, 0x444444, 0x222222);
        gridHelper.position.y = -0.01;
        this.scene.add(gridHelper);

        const groundGeo = new THREE.PlaneGeometry(100, 100);
        const groundMat = new THREE.MeshStandardMaterial({
            color: 0x8b7355,
            roughness: 0.9,
            metalness: 0.1
        });
        const ground = new THREE.Mesh(groundGeo, groundMat);
        ground.rotation.x = -Math.PI / 2;
        ground.receiveShadow = true;
        this.scene.add(ground);
    }

    _buildChariot() {
        this.chariotGroup = new THREE.Group();
        this.scene.add(this.chariotGroup);

        const floorGeo = new THREE.BoxGeometry(CHARIOT_WHEELBASE * 0.8, 0.08, CHARIOT_TRACK_WIDTH);
        const woodMat = this.getSharedMaterial('wood', () =>
            new THREE.MeshStandardMaterial({ color: 0x8b4513, roughness: 0.7, metalness: 0.1 })
        );
        const floor = new THREE.Mesh(floorGeo, woodMat);
        floor.position.y = WHEEL_RADIUS + 0.04;
        floor.castShadow = true;
        floor.receiveShadow = true;
        this.chariotGroup.add(floor);

        const bodyMat = this.getSharedMaterial('body', () =>
            new THREE.MeshStandardMaterial({ color: 0xa0522d, roughness: 0.6, metalness: 0.1 })
        );
        const bodyFront = new THREE.Mesh(
            new THREE.BoxGeometry(0.05, 0.5, CHARIOT_TRACK_WIDTH), bodyMat
        );
        bodyFront.position.set(CHARIOT_WHEELBASE * 0.3, WHEEL_RADIUS + 0.33, 0);
        bodyFront.castShadow = true;
        this.chariotGroup.add(bodyFront);

        const bodyBack = new THREE.Mesh(
            new THREE.BoxGeometry(0.05, 0.6, CHARIOT_TRACK_WIDTH), bodyMat
        );
        bodyBack.position.set(-CHARIOT_WHEELBASE * 0.4, WHEEL_RADIUS + 0.38, 0);
        bodyBack.castShadow = true;
        this.chariotGroup.add(bodyBack);

        this._buildRailing();
        this._buildFrontWheels();
        this._buildRearWheels();
        this._buildPole();
        this._buildLinkage();
    }

    _buildRailing() {
        const railingMat = this.getSharedMaterial('railing', () =>
            new THREE.MeshStandardMaterial({ color: 0x654321, roughness: 0.7, metalness: 0.1 })
        );
        const postGeo = this.getSharedGeometry('postGeo', () =>
            new THREE.CylinderGeometry(0.015, 0.02, 0.55, 8)
        );

        const postPositions = [
            [0.8, 0.9], [0.8, -0.9], [-0.8, 0.9], [-0.8, -0.9],
            [0.3, 0.9], [0.3, -0.9], [-0.3, 0.9], [-0.3, -0.9]
        ];

        const railingPosts = this.createInstancedMesh(
            postGeo, railingMat, postPositions.length, (i, obj) => {
                const [x, z] = postPositions[i];
                obj.position.set(x, WHEEL_RADIUS + 0.355, z);
            }
        );
        railingPosts.castShadow = true;
        this.chariotGroup.add(railingPosts);

        const railGeo = new THREE.BoxGeometry(CHARIOT_WHEELBASE * 0.85, 0.03, 0.03);
        const sideRailLeft = new THREE.Mesh(railGeo, railingMat);
        sideRailLeft.position.set(0, WHEEL_RADIUS + 0.355, CHARIOT_TRACK_WIDTH / 2 - 0.05);
        this.chariotGroup.add(sideRailLeft);

        const sideRailRight = new THREE.Mesh(railGeo, railingMat);
        sideRailRight.position.set(0, WHEEL_RADIUS + 0.355, -CHARIOT_TRACK_WIDTH / 2 + 0.05);
        this.chariotGroup.add(sideRailRight);

        const backRailGeo = new THREE.BoxGeometry(0.03, 0.03, CHARIOT_TRACK_WIDTH - 0.1);
        const backRail = new THREE.Mesh(backRailGeo, railingMat);
        backRail.position.set(-CHARIOT_WHEELBASE * 0.4, WHEEL_RADIUS + 0.355, 0);
        this.chariotGroup.add(backRail);
    }

    _buildFrontWheels() {
        this.leftKingpin = new THREE.Group();
        this.leftKingpin.position.set(CHARIOT_WHEELBASE * 0.4, WHEEL_RADIUS, CHARIOT_TRACK_WIDTH / 2);
        this.chariotGroup.add(this.leftKingpin);

        this.rightKingpin = new THREE.Group();
        this.rightKingpin.position.set(CHARIOT_WHEELBASE * 0.4, WHEEL_RADIUS, -CHARIOT_TRACK_WIDTH / 2);
        this.chariotGroup.add(this.rightKingpin);

        const wheelFrontLeft = this._createWheelGroup();
        this.leftKingpin.add(wheelFrontLeft);
        this.wheelGroups.frontLeft = wheelFrontLeft;

        const wheelFrontRight = this._createWheelGroup();
        this.rightKingpin.add(wheelFrontRight);
        this.wheelGroups.frontRight = wheelFrontRight;
    }

    _buildRearWheels() {
        const wheelRearLeft = this._createWheelGroup();
        wheelRearLeft.position.set(-CHARIOT_WHEELBASE * 0.4, 0, CHARIOT_TRACK_WIDTH / 2);
        this.chariotGroup.add(wheelRearLeft);
        this.wheelGroups.rearLeft = wheelRearLeft;

        const wheelRearRight = this._createWheelGroup();
        wheelRearRight.position.set(-CHARIOT_WHEELBASE * 0.4, 0, -CHARIOT_TRACK_WIDTH / 2);
        this.chariotGroup.add(wheelRearRight);
        this.wheelGroups.rearRight = wheelRearRight;
    }

    _createWheelGroup() {
        const wheelGroup = new THREE.Group();
        wheelGroup.userData.spinAngle = 0;

        const tireGeo = this.getSharedGeometry('tireGeo', () =>
            new THREE.TorusGeometry(WHEEL_RADIUS, 0.06, 16, 32)
        );
        const tireMat = this.getSharedMaterial('tireMat', () =>
            new THREE.MeshStandardMaterial({ color: 0x2c1810, roughness: 0.9 })
        );
        const tire = new THREE.Mesh(tireGeo, tireMat);
        tire.rotation.y = Math.PI / 2;
        tire.castShadow = true;
        wheelGroup.add(tire);

        const rimGeo = this.getSharedGeometry('rimGeo', () =>
            new THREE.CylinderGeometry(WHEEL_RADIUS * 0.3, WHEEL_RADIUS * 0.3, 0.08, 16)
        );
        const woodMat = this.getSharedMaterial('wood', () =>
            new THREE.MeshStandardMaterial({ color: 0x8b4513, roughness: 0.7, metalness: 0.1 })
        );
        const rim = new THREE.Mesh(rimGeo, woodMat);
        rim.rotation.z = Math.PI / 2;
        wheelGroup.add(rim);

        const spokeGeom = this.getSharedGeometry('spokeGeo', () =>
            new THREE.BoxGeometry(0.04, WHEEL_RADIUS * 1.5, 0.04)
        );
        const spokeMat = this.getSharedMaterial('spokeMat', () =>
            new THREE.MeshStandardMaterial({ color: 0xd2b48c, roughness: 0.6 })
        );
        const spokes = this.createInstancedMesh(
            spokeGeom, spokeMat, NUM_SPOKES, (i, obj) => {
                obj.rotation.set(0, (i * Math.PI) / NUM_SPOKES, 0);
            }
        );
        spokes.castShadow = true;
        wheelGroup.add(spokes);
        wheelGroup.userData.spokes = spokes;
        this._spokeMeshes.push(spokes);

        const hubGeo = this.getSharedGeometry('hubGeo', () =>
            new THREE.CylinderGeometry(0.04, 0.05, 0.12, 12)
        );
        const copperMat = this.getSharedMaterial('copperMat', () =>
            new THREE.MeshStandardMaterial({ color: 0xb87333, roughness: 0.3, metalness: 0.8 })
        );
        const hub = new THREE.Mesh(hubGeo, copperMat);
        hub.rotation.z = Math.PI / 2;
        wheelGroup.add(hub);

        return wheelGroup;
    }

    _buildPole() {
        this.poleGroup = new THREE.Group();
        this.poleGroup.position.set(CHARIOT_WHEELBASE * 0.4, WHEEL_RADIUS, 0);
        this.chariotGroup.add(this.poleGroup);

        const poleGeo = new THREE.CylinderGeometry(0.04, 0.05, POLE_LENGTH, 12);
        const poleMat = new THREE.MeshStandardMaterial({
            color: 0x654321, roughness: 0.7, metalness: 0.1
        });
        const pole = new THREE.Mesh(poleGeo, poleMat);
        pole.rotation.z = Math.PI / 2;
        pole.position.x = POLE_LENGTH / 2 - 0.1;
        pole.castShadow = true;
        this.poleGroup.add(pole);

        const yokeGeo = new THREE.TorusGeometry(0.2, 0.03, 12, 24);
        const yokeMat = new THREE.MeshStandardMaterial({
            color: 0xb87333, roughness: 0.3, metalness: 0.8
        });
        const yoke = new THREE.Mesh(yokeGeo, yokeMat);
        yoke.position.set(POLE_LENGTH - 0.15, 0, 0);
        this.poleGroup.add(yoke);
    }

    _buildLinkage() {
        const linkageMat = new THREE.MeshStandardMaterial({
            color: 0x8b4513, roughness: 0.6, metalness: 0.2
        });

        const leftTieRod = new THREE.Mesh(
            new THREE.CylinderGeometry(0.02, 0.02, CHARIOT_TRACK_WIDTH / 2, 8),
            linkageMat
        );
        leftTieRod.rotation.z = Math.PI / 2;
        leftTieRod.position.set(
            CHARIOT_WHEELBASE * 0.4 - 0.15, WHEEL_RADIUS - 0.05, CHARIOT_TRACK_WIDTH / 4
        );
        leftTieRod.castShadow = true;
        this.chariotGroup.add(leftTieRod);

        const rightTieRod = new THREE.Mesh(
            new THREE.CylinderGeometry(0.02, 0.02, CHARIOT_TRACK_WIDTH / 2, 8),
            linkageMat
        );
        rightTieRod.rotation.z = Math.PI / 2;
        rightTieRod.position.set(
            CHARIOT_WHEELBASE * 0.4 - 0.15, WHEEL_RADIUS - 0.05, -CHARIOT_TRACK_WIDTH / 4
        );
        rightTieRod.castShadow = true;
        this.chariotGroup.add(rightTieRod);
    }

    _setupTrajectory() {
        const trajMat = new THREE.LineBasicMaterial({ color: 0xff4500, linewidth: 2 });
        const trajGeo = new THREE.BufferGeometry();
        trajGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
        this.trajectoryLine = new THREE.Line(trajGeo, trajMat);
        this.scene.add(this.trajectoryLine);

        const innerMat = new THREE.LineBasicMaterial({ color: 0x32cd32, linewidth: 2 });
        const innerGeo = new THREE.BufferGeometry();
        innerGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
        this.innerTrajectoryLine = new THREE.Line(innerGeo, innerMat);
        this.scene.add(this.innerTrajectoryLine);
    }

    updateSteering(poleAngleDeg, steeringData) {
        this.currentPoleAngle = poleAngleDeg;
        const poleAngleRad = (poleAngleDeg * Math.PI) / 180;

        if (steeringData && typeof steeringData.turning_radius === 'number') {
            const R = steeringData.turning_radius;
            const L = CHARIOT_WHEELBASE;
            const T = CHARIOT_TRACK_WIDTH;

            const innerAngle = Math.atan(L / (R - T / 2));
            const outerAngle = Math.atan(L / (R + T / 2));

            if (poleAngleDeg > 0) {
                this.leftKingpin.rotation.y = -outerAngle;
                this.rightKingpin.rotation.y = -innerAngle;
            } else if (poleAngleDeg < 0) {
                this.leftKingpin.rotation.y = -innerAngle;
                this.rightKingpin.rotation.y = -outerAngle;
            } else {
                this.leftKingpin.rotation.y = 0;
                this.rightKingpin.rotation.y = 0;
            }
        } else {
            this.leftKingpin.rotation.y = -poleAngleRad * 0.85;
            this.rightKingpin.rotation.y = -poleAngleRad * 0.72;
        }

        this.poleGroup.rotation.y = -poleAngleRad * 0.8;
        this._updateLinkageVisual(poleAngleRad);
    }

    _updateLinkageVisual(poleAngleRad) {
        this.chariotGroup.children.forEach(obj => {
            if (obj.material && obj.material.color) {
                if (obj.material.color.getHex() === 0x8b4513 && obj.geometry &&
                    obj.geometry.type === 'CylinderGeometry' &&
                    obj.position.y < WHEEL_RADIUS) {
                    obj.position.y = WHEEL_RADIUS - 0.05 + Math.sin(poleAngleRad) * 0.02;
                }
            }
        });
    }

    setVehicleSpeed(speed) {
        this.currentVehicleSpeed = speed;
    }

    updateWheelRotation(innerFactor, outerFactor) {
        const dt = Math.min(this.clock.getDelta(), 0.05);
        const spinBase = (this.currentVehicleSpeed * dt) / WHEEL_RADIUS;

        const leftFactor = this.currentPoleAngle > 0 ? outerFactor : innerFactor;
        const rightFactor = this.currentPoleAngle > 0 ? innerFactor : outerFactor;

        if (this.wheelGroups.frontLeft) {
            this._spinWheel(this.wheelGroups.frontLeft, spinBase * leftFactor);
        }
        if (this.wheelGroups.frontRight) {
            this._spinWheel(this.wheelGroups.frontRight, spinBase * rightFactor);
        }
        if (this.wheelGroups.rearLeft) {
            this._spinWheel(this.wheelGroups.rearLeft, spinBase * 0.98);
        }
        if (this.wheelGroups.rearRight) {
            this._spinWheel(this.wheelGroups.rearRight, spinBase * 0.98);
        }
    }

    _spinWheel(wheelGroup, angle) {
        wheelGroup.userData.spinAngle += angle;
        const spokes = wheelGroup.userData.spokes;
        if (!spokes) return;

        const dummy = new THREE.Object3D();
        for (let i = 0; i < NUM_SPOKES; i++) {
            dummy.rotation.set(0, (i * Math.PI) / NUM_SPOKES + wheelGroup.userData.spinAngle, 0);
            dummy.updateMatrix();
            spokes.setMatrixAt(i, dummy.matrix);
        }
        spokes.instanceMatrix.needsUpdate = true;
    }

    updateTrajectory(steeringData, speed) {
        if (!steeringData || speed < 0.5) return;

        const dt = Math.min((performance.now() - this.lastTime) / 1000, 0.05);
        const R = steeringData.turning_radius;
        const omega = speed / R;
        const L = CHARIOT_WHEELBASE;
        const T = CHARIOT_TRACK_WIDTH;

        const center = new THREE.Vector3(
            this.chariotGroup.position.x,
            0.02,
            this.chariotGroup.position.z
        );
        this.trajectoryPoints.push(center.clone());

        const innerAngle = this.currentPoleAngle > 0
            ? steeringData.inner_wheel_angle * Math.PI / 180
            : steeringData.outer_wheel_angle * Math.PI / 180;
        const innerR = L / Math.tan(Math.abs(innerAngle)) - T / 2;
        const innerPt = this.currentPoleAngle > 0
            ? new THREE.Vector3(center.x, 0.02, center.z + T / 2)
            : new THREE.Vector3(center.x, 0.02, center.z - T / 2);
        this.innerTrajectoryPoints.push(innerPt.clone());

        if (this.trajectoryPoints.length > MAX_POINTS) this.trajectoryPoints.shift();
        if (this.innerTrajectoryPoints.length > MAX_POINTS) this.innerTrajectoryPoints.shift();

        this._updateTrajectoryLine(this.trajectoryLine, this.trajectoryPoints);
        this._updateTrajectoryLine(this.innerTrajectoryLine, this.innerTrajectoryPoints);
    }

    _updateTrajectoryLine(line, points) {
        const positions = line.geometry.attributes.position.array;
        for (let i = 0; i < points.length; i++) {
            positions[i * 3] = points[i].x;
            positions[i * 3 + 1] = points[i].y;
            positions[i * 3 + 2] = points[i].z;
        }
        line.geometry.attributes.position.needsUpdate = true;
        line.geometry.setDrawRange(0, points.length);
    }

    setRolloverRisk(risk) {
        const bodyMat = this._materialCache.get('body');
        if (bodyMat) {
            if (risk > 70) {
                bodyMat.color.setHex(0xff0000);
            } else if (risk > 40) {
                bodyMat.color.setHex(0xffa500);
            } else {
                bodyMat.color.setHex(0xa0522d);
            }
        }
    }

    setCameraView(view) {
        if (view === 'front') {
            this.camera.position.set(0, 3, 8);
        } else if (view === 'side') {
            this.camera.position.set(8, 2, 0);
        } else if (view === 'top') {
            this.camera.position.set(0, 10, 0.1);
        } else {
            this.camera.position.set(5, 4, 6);
        }
        this.camera.lookAt(0, 0.5, 0);
    }

    _onResize() {
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        this.renderer.render(this.scene, this.camera);
    }

    clearTrajectories() {
        this.trajectoryPoints = [];
        this.innerTrajectoryPoints = [];
        this._updateTrajectoryLine(this.trajectoryLine, []);
        this._updateTrajectoryLine(this.innerTrajectoryLine, []);
    }
}

window.Chariot3D = Chariot3D;
