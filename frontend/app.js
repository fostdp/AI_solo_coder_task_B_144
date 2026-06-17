import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';


const API_BASE = window.location.origin;
let socket = null;
let currentVehicleId = 'chariot-qin-001';


let scene, camera, renderer, controls;
let chariotGroup;
let leftWheel, rightWheel, frontLeftWheel, frontRightWheel;
let wheelGroups = [];
let poleGroup;
let leftTieRod, rightTieRod;
let leftKingpin, rightKingpin;
let wheelTrajectoryLine, innerTrajectoryLine;
let trajectoryPoints = [];
let innerTrajectoryPoints = [];
let animationId;

let geometryCache = {};
let materialCache = {};
let instancedMeshes = [];

const dummyObj = new THREE.Object3D();
const tmpQuat = new THREE.Quaternion();
const tmpScale = new THREE.Vector3(1, 1, 1);

const CHARIOT_WHEELBASE = 2.5;
const CHARIOT_TRACK_WIDTH = 1.8;
const WHEEL_RADIUS = 0.35;
const WHEEL_WIDTH = 0.15;
const POLE_LENGTH = 1.8;
const NUM_SPOKES = 8;
const NUM_RAILING_POSTS = 8;


function getSharedGeometry(key, factory) {
    if (!geometryCache[key]) {
        geometryCache[key] = factory();
    }
    return geometryCache[key];
}


function getSharedMaterial(key, factory) {
    if (!materialCache[key]) {
        materialCache[key] = factory();
    }
    return materialCache[key];
}


function createInstancedMesh(geometry, material, count, setupCallback) {
    const instanced = new THREE.InstancedMesh(geometry, material, count);
    instanced.castShadow = true;
    instanced.receiveShadow = true;
    instanced.instanceMatrix.setUsage(THREE.DynamicDrawUsage);

    if (setupCallback) {
        for (let i = 0; i < count; i++) {
            setupCallback(i, dummyObj);
            dummyObj.updateMatrix();
            instanced.setMatrixAt(i, dummyObj.matrix);
        }
    }
    instanced.instanceMatrix.needsUpdate = true;
    instancedMeshes.push(instanced);
    return instanced;
}


function initThreeJS() {
    const container = document.getElementById('canvasContainer');
    const width = container.clientWidth;
    const height = container.clientHeight;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a15);
    scene.fog = new THREE.Fog(0x0a0a15, 20, 50);

    camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 100);
    camera.position.set(4, 3, 5);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({
        antialias: true,
        alpha: true,
        powerPreference: "high-performance"
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.appendChild(renderer.domElement);

    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.minDistance = 2;
    controls.maxDistance = 20;
    controls.maxPolarAngle = Math.PI / 2 + 0.1;

    addLights();
    addGround();
    createChariot();
    createTrajectoryLine();

    window.addEventListener('resize', onWindowResize);
    animate();
}


function addLights() {
    const ambientLight = new THREE.AmbientLight(0x404060, 0.6);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.85);
    directionalLight.position.set(5, 10, 5);
    directionalLight.castShadow = true;
    directionalLight.shadow.mapSize.width = 1024;
    directionalLight.shadow.mapSize.height = 1024;
    directionalLight.shadow.camera.near = 0.5;
    directionalLight.shadow.camera.far = 50;
    directionalLight.shadow.camera.left = -10;
    directionalLight.shadow.camera.right = 10;
    directionalLight.shadow.camera.top = 10;
    directionalLight.shadow.camera.bottom = -10;
    scene.add(directionalLight);

    const pointLight = new THREE.PointLight(0x00d9ff, 0.5, 10);
    pointLight.position.set(0, 2, 3);
    scene.add(pointLight);
}


function addGround() {
    const gridHelper = new THREE.GridHelper(20, 20, 0x00d9ff, 0x0f3460);
    gridHelper.position.y = -WHEEL_RADIUS;
    gridHelper.material.opacity = 0.3;
    gridHelper.material.transparent = true;
    scene.add(gridHelper);

    const groundGeometry = new THREE.PlaneGeometry(30, 30);
    const groundMaterial = new THREE.MeshStandardMaterial({
        color: 0x1a1a2e,
        roughness: 0.9,
        metalness: 0.1
    });
    const ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -WHEEL_RADIUS - 0.01;
    ground.receiveShadow = true;
    scene.add(ground);
}


function createWheelGroup(radius, width) {
    const group = new THREE.Group();
    group.userData.spinAngle = 0;

    const tireGeom = getSharedGeometry('tire', () =>
        new THREE.TorusGeometry(radius, width * 0.3, 8, 32)
    );
    const tireMat = getSharedMaterial('tire', () =>
        new THREE.MeshStandardMaterial({ color: 0x2c1810, roughness: 0.8, metalness: 0.2 })
    );
    const tire = new THREE.Mesh(tireGeom, tireMat);
    tire.rotation.y = Math.PI / 2;
    tire.castShadow = true;
    group.add(tire);

    const rimGeom = getSharedGeometry('rim', () =>
        new THREE.CylinderGeometry(radius * 0.7, radius * 0.7, width * 0.6, 16)
    );
    const rimMat = getSharedMaterial('wood_rim', () =>
        new THREE.MeshStandardMaterial({ color: 0x8b4513, roughness: 0.6, metalness: 0.3 })
    );
    const rim = new THREE.Mesh(rimGeom, rimMat);
    rim.rotation.z = Math.PI / 2;
    rim.castShadow = true;
    group.add(rim);

    const spokeGeom = getSharedGeometry('spoke', () =>
        new THREE.BoxGeometry(radius * 0.65, 0.04, 0.04)
    );
    const spokeMat = getSharedMaterial('wood_spoke', () =>
        new THREE.MeshStandardMaterial({ color: 0x654321, roughness: 0.7 })
    );

    const spokes = createInstancedMesh(spokeGeom, spokeMat, NUM_SPOKES, (i, obj) => {
        obj.position.set(0, 0, 0);
        obj.rotation.set(0, (i * Math.PI) / NUM_SPOKES, 0);
        obj.scale.set(1, 1, 1);
    });
    group.add(spokes);
    group.userData.spokes = spokes;

    const hubGeom = getSharedGeometry('hub', () =>
        new THREE.CylinderGeometry(0.08, 0.08, width * 0.8, 12)
    );
    const hubMat = getSharedMaterial('brass', () =>
        new THREE.MeshStandardMaterial({ color: 0xd4af37, roughness: 0.4, metalness: 0.8 })
    );
    const hub = new THREE.Mesh(hubGeom, hubMat);
    hub.rotation.z = Math.PI / 2;
    hub.castShadow = true;
    group.add(hub);

    return group;
}


function createChariot() {
    chariotGroup = new THREE.Group();
    scene.add(chariotGroup);

    const bodyLength = CHARIOT_WHEELBASE + 0.6;
    const bodyWidth = CHARIOT_TRACK_WIDTH * 0.9;
    const bodyHeight = 0.3;

    const bodyGeom = new THREE.BoxGeometry(bodyWidth, bodyHeight, bodyLength);
    const bodyMat = getSharedMaterial('wood_body', () =>
        new THREE.MeshStandardMaterial({ color: 0x8b4513, roughness: 0.7, metalness: 0.1 })
    );
    const body = new THREE.Mesh(bodyGeom, bodyMat);
    body.position.y = WHEEL_RADIUS + bodyHeight / 2;
    body.castShadow = true;
    body.receiveShadow = true;
    chariotGroup.add(body);

    const floorGeom = new THREE.BoxGeometry(bodyWidth * 0.9, 0.05, bodyLength * 0.85);
    const floorMat = getSharedMaterial('wood_floor', () =>
        new THREE.MeshStandardMaterial({ color: 0x654321, roughness: 0.8 })
    );
    const floor = new THREE.Mesh(floorGeom, floorMat);
    floor.position.y = WHEEL_RADIUS + bodyHeight + 0.02;
    floor.castShadow = true;
    chariotGroup.add(floor);

    const railingMat = getSharedMaterial('wood_railing', () =>
        new THREE.MeshStandardMaterial({ color: 0x654321, roughness: 0.7 })
    );

    const postGeom = getSharedGeometry('railing_post', () =>
        new THREE.CylinderGeometry(0.03, 0.03, 0.6, 8)
    );

    const postPositions = [];
    for (const side of [-1, 1]) {
        for (const z of [-bodyLength * 0.3, 0, bodyLength * 0.3]) {
            postPositions.push({
                x: side * bodyWidth * 0.4,
                y: WHEEL_RADIUS + bodyHeight + 0.3,
                z: z
            });
        }
    }
    for (const side of [-1, 1]) {
        postPositions.push({
            x: side * bodyWidth * 0.3,
            y: WHEEL_RADIUS + bodyHeight + 0.4,
            z: -bodyLength * 0.4
        });
    }

    const railingPosts = createInstancedMesh(postGeom, railingMat, postPositions.length, (i, obj) => {
        const p = postPositions[i];
        obj.position.set(p.x, p.y, p.z);
        obj.rotation.set(0, 0, 0);
        obj.scale.set(1, 1, 1);
    });
    chariotGroup.add(railingPosts);

    const sideRailGeom = getSharedGeometry('side_rail', () =>
        new THREE.BoxGeometry(0.04, 0.04, bodyLength * 0.7)
    );
    const sideRailLeft = new THREE.Mesh(sideRailGeom, railingMat);
    sideRailLeft.position.set(-bodyWidth * 0.4, WHEEL_RADIUS + bodyHeight + 0.6, 0);
    sideRailLeft.castShadow = true;
    chariotGroup.add(sideRailLeft);

    const sideRailRight = new THREE.Mesh(sideRailGeom, railingMat);
    sideRailRight.position.set(bodyWidth * 0.4, WHEEL_RADIUS + bodyHeight + 0.6, 0);
    sideRailRight.castShadow = true;
    chariotGroup.add(sideRailRight);

    const backRailGeom = new THREE.BoxGeometry(bodyWidth * 0.7, 0.04, 0.04);
    const backRail = new THREE.Mesh(backRailGeom, railingMat);
    backRail.position.set(0, WHEEL_RADIUS + bodyHeight + 0.8, -bodyLength * 0.4);
    backRail.castShadow = true;
    chariotGroup.add(backRail);

    leftKingpin = new THREE.Group();
    leftKingpin.position.set(-CHARIOT_TRACK_WIDTH / 2, 0, CHARIOT_WHEELBASE / 2 - 0.2);
    chariotGroup.add(leftKingpin);

    frontLeftWheel = createWheelGroup(WHEEL_RADIUS, WHEEL_WIDTH);
    frontLeftWheel.position.set(0, WHEEL_RADIUS, 0);
    leftKingpin.add(frontLeftWheel);
    wheelGroups.push(frontLeftWheel);

    rightKingpin = new THREE.Group();
    rightKingpin.position.set(CHARIOT_TRACK_WIDTH / 2, 0, CHARIOT_WHEELBASE / 2 - 0.2);
    chariotGroup.add(rightKingpin);

    frontRightWheel = createWheelGroup(WHEEL_RADIUS, WHEEL_WIDTH);
    frontRightWheel.position.set(0, WHEEL_RADIUS, 0);
    rightKingpin.add(frontRightWheel);
    wheelGroups.push(frontRightWheel);

    leftWheel = createWheelGroup(WHEEL_RADIUS, WHEEL_WIDTH);
    leftWheel.position.set(-CHARIOT_TRACK_WIDTH / 2, WHEEL_RADIUS, -CHARIOT_WHEELBASE / 2 + 0.2);
    chariotGroup.add(leftWheel);
    wheelGroups.push(leftWheel);

    rightWheel = createWheelGroup(WHEEL_RADIUS, WHEEL_WIDTH);
    rightWheel.position.set(CHARIOT_TRACK_WIDTH / 2, WHEEL_RADIUS, -CHARIOT_WHEELBASE / 2 + 0.2);
    chariotGroup.add(rightWheel);
    wheelGroups.push(rightWheel);

    createSteeringLinkage();

    poleGroup = new THREE.Group();
    poleGroup.position.set(0, WHEEL_RADIUS + bodyHeight * 0.3, CHARIOT_WHEELBASE / 2 + 0.3);
    chariotGroup.add(poleGroup);

    const poleGeom = new THREE.BoxGeometry(0.08, 0.1, POLE_LENGTH);
    const poleMat = getSharedMaterial('wood_pole', () =>
        new THREE.MeshStandardMaterial({ color: 0x654321, roughness: 0.6 })
    );
    const pole = new THREE.Mesh(poleGeom, poleMat);
    pole.position.set(0, 0, POLE_LENGTH / 2);
    pole.castShadow = true;
    poleGroup.add(pole);

    const yokeGeom = new THREE.BoxGeometry(0.6, 0.15, 0.2);
    const yokeMat = getSharedMaterial('brass');
    const yoke = new THREE.Mesh(yokeGeom, yokeMat);
    yoke.position.set(0, 0, POLE_LENGTH + 0.1);
    yoke.castShadow = true;
    poleGroup.add(yoke);

    chariotGroup.position.z = -CHARIOT_WHEELBASE / 4;
}


function createSteeringLinkage() {
    const linkageMat = getSharedMaterial('brass');

    const tieRodLength = CHARIOT_TRACK_WIDTH * 0.85;
    const tieRodGeom = getSharedGeometry('tie_rod', () =>
        new THREE.CylinderGeometry(0.025, 0.025, tieRodLength, 8)
    );

    leftTieRod = new THREE.Mesh(tieRodGeom, linkageMat);
    leftTieRod.rotation.z = Math.PI / 2;
    leftTieRod.position.set(-tieRodLength / 2, WHEEL_RADIUS * 0.5, CHARIOT_WHEELBASE / 2 - 0.3);
    chariotGroup.add(leftTieRod);

    rightTieRod = new THREE.Mesh(tieRodGeom, linkageMat);
    rightTieRod.rotation.z = Math.PI / 2;
    rightTieRod.position.set(tieRodLength / 2, WHEEL_RADIUS * 0.5, CHARIOT_WHEELBASE / 2 - 0.3);
    chariotGroup.add(rightTieRod);

    const armGeom = getSharedGeometry('steering_arm', () =>
        new THREE.BoxGeometry(0.04, 0.04, 0.3)
    );
    const leftArm = new THREE.Mesh(armGeom, linkageMat);
    leftArm.position.set(0, 0, 0.15);
    leftKingpin.add(leftArm);

    const rightArm = new THREE.Mesh(armGeom, linkageMat);
    rightArm.position.set(0, 0, 0.15);
    rightKingpin.add(rightArm);
}


function createTrajectoryLine() {
    const centerMat = new THREE.LineBasicMaterial({
        color: 0x00d9ff,
        linewidth: 2,
        opacity: 0.8,
        transparent: true
    });
    const centerGeom = new THREE.BufferGeometry();
    wheelTrajectoryLine = new THREE.Line(centerGeom, centerMat);
    wheelTrajectoryLine.position.y = -WHEEL_RADIUS + 0.02;
    scene.add(wheelTrajectoryLine);

    const innerMat = new THREE.LineBasicMaterial({
        color: 0xe94560,
        linewidth: 2,
        opacity: 0.6,
        transparent: true
    });
    const innerGeom = new THREE.BufferGeometry();
    innerTrajectoryLine = new THREE.Line(innerGeom, innerMat);
    innerTrajectoryLine.position.y = -WHEEL_RADIUS + 0.02;
    scene.add(innerTrajectoryLine);
}


function updateSteering(poleAngleDeg) {
    if (!leftKingpin || !rightKingpin) return;

    const poleAngleRad = THREE.MathUtils.degToRad(poleAngleDeg);
    const L = CHARIOT_WHEELBASE;
    const T = CHARIOT_TRACK_WIDTH;

    if (Math.abs(poleAngleRad) < 0.001) {
        leftKingpin.rotation.y = 0;
        rightKingpin.rotation.y = 0;
    } else {
        const R = L / Math.tan(Math.abs(poleAngleRad));
        const innerAngle = Math.atan(L / (R - T / 2));
        const outerAngle = Math.atan(L / (R + T / 2));

        if (poleAngleDeg > 0) {
            leftKingpin.rotation.y = outerAngle;
            rightKingpin.rotation.y = innerAngle;
        } else {
            leftKingpin.rotation.y = -innerAngle;
            rightKingpin.rotation.y = -outerAngle;
        }
    }

    if (poleGroup) {
        poleGroup.rotation.y = poleAngleRad * 0.8;
    }

    updateLinkageVisual(poleAngleDeg);
}


function updateLinkageVisual(poleAngleDeg) {
    const poleAngleRad = THREE.MathUtils.degToRad(poleAngleDeg);
    if (leftTieRod && rightTieRod) {
        const offset = Math.sin(poleAngleRad) * 0.3;
        leftTieRod.position.y = WHEEL_RADIUS * 0.5 + offset * 0.1;
        rightTieRod.position.y = WHEEL_RADIUS * 0.5 + offset * 0.1;
    }
}


function updateWheelRotation(speed, dt) {
    const angularSpeed = speed / WHEEL_RADIUS;
    for (const wg of wheelGroups) {
        if (!wg || !wg.children || wg.children.length === 0) continue;

        wg.userData.spinAngle = (wg.userData.spinAngle || 0) + angularSpeed * dt;
        const tire = wg.children[0];
        const rim = wg.children[1];
        if (tire) tire.rotation.x = wg.userData.spinAngle;
        if (rim) rim.rotation.z = Math.PI / 2 + wg.userData.spinAngle;

        const spokes = wg.userData.spokes;
        if (spokes) {
            for (let i = 0; i < NUM_SPOKES; i++) {
                dummyObj.position.set(0, 0, 0);
                dummyObj.rotation.set(0, (i * Math.PI) / NUM_SPOKES + wg.userData.spinAngle, 0);
                dummyObj.scale.copy(tmpScale);
                dummyObj.updateMatrix();
                spokes.setMatrixAt(i, dummyObj.matrix);
            }
            spokes.instanceMatrix.needsUpdate = true;
        }
    }
}


function updateTrajectories(poleAngleDeg, speed, dt) {
    if (!wheelTrajectoryLine) return;

    const centerPt = new THREE.Vector3(
        chariotGroup.position.x,
        -WHEEL_RADIUS + 0.02,
        chariotGroup.position.z
    );

    const R = Math.abs(THREE.MathUtils.degToRad(poleAngleDeg)) < 0.001
        ? 999999
        : CHARIOT_WHEELBASE / Math.tan(Math.abs(THREE.MathUtils.degToRad(poleAngleDeg)) * 0.85);
    const direction = poleAngleDeg >= 0 ? 1 : -1;

    const innerPt = new THREE.Vector3(
        chariotGroup.position.x - direction * CHARIOT_TRACK_WIDTH / 2 * 0.5,
        -WHEEL_RADIUS + 0.02,
        chariotGroup.position.z
    );

    trajectoryPoints.push(centerPt);
    innerTrajectoryPoints.push(innerPt);

    const MAX_POINTS = 400;
    if (trajectoryPoints.length > MAX_POINTS) trajectoryPoints.shift();
    if (innerTrajectoryPoints.length > MAX_POINTS) innerTrajectoryPoints.shift();

    function updateLineGeom(line, points) {
        const positions = new Float32Array(points.length * 3);
        for (let i = 0; i < points.length; i++) {
            positions[i * 3] = points[i].x;
            positions[i * 3 + 1] = points[i].y;
            positions[i * 3 + 2] = points[i].z;
        }
        line.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        line.geometry.computeBoundingSphere();
    }

    updateLineGeom(wheelTrajectoryLine, trajectoryPoints);
    if (innerTrajectoryLine) updateLineGeom(innerTrajectoryLine, innerTrajectoryPoints);
}


function updateChariotPosition(poleAngleDeg, speed, dt) {
    const poleAngleRad = THREE.MathUtils.degToRad(poleAngleDeg) * 0.85;

    if (Math.abs(poleAngleRad) < 0.001) {
        chariotGroup.position.z -= speed * dt;
    } else {
        const R = CHARIOT_WHEELBASE / Math.tan(poleAngleRad);
        const angularVel = speed / R;
        const centerX = chariotGroup.position.x + R;
        const centerZ = chariotGroup.position.z;
        const angle = angularVel * dt;
        const cos = Math.cos(angle);
        const sin = Math.sin(angle);

        chariotGroup.position.x = centerX - R * cos;
        chariotGroup.position.z = centerZ + R * sin;
        chariotGroup.rotation.y += angle;
    }
}


function onWindowResize() {
    const container = document.getElementById('canvasContainer');
    const width = container.clientWidth;
    const height = container.clientHeight;

    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}


let lastTime = 0;
function animate(time) {
    const dt = Math.min((time - lastTime) / 1000, 0.05);
    lastTime = time;

    const speed = parseFloat(document.getElementById('speedSlider')?.value) || 5;
    const poleAngle = parseFloat(document.getElementById('poleAngleSlider')?.value) || 0;

    updateSteering(poleAngle);
    updateWheelRotation(speed * 0.5, dt);
    updateChariotPosition(poleAngle, speed * 0.3, dt);
    updateTrajectories(poleAngle, speed, dt);

    controls.update();
    renderer.render(scene, camera);

    animationId = requestAnimationFrame(animate);
}


function setView(view) {
    if (view === '3d') {
        camera.position.set(4, 3, 5);
    } else if (view === 'top') {
        camera.position.set(0, 10, 0.1);
    } else if (view === 'side') {
        camera.position.set(6, 1.5, 0);
    }
    camera.lookAt(0, 0, 0);
}


function drawLinkageDiagram(poleAngleDeg) {
    const canvas = document.getElementById('linkageCanvas');
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    ctx.strokeStyle = '#0f3460';
    ctx.lineWidth = 1;
    for (let i = 0; i < w; i += 50) {
        ctx.beginPath();
        ctx.moveTo(i, 0);
        ctx.lineTo(i, h);
        ctx.stroke();
    }
    for (let i = 0; i < h; i += 50) {
        ctx.beginPath();
        ctx.moveTo(0, i);
        ctx.lineTo(w, i);
        ctx.stroke();
    }

    const scale = 80;
    const centerX = w / 2;
    const centerY = h * 0.6;

    const wheelbase = CHARIOT_WHEELBASE * scale;
    const trackWidth = CHARIOT_TRACK_WIDTH * scale;
    const poleLen = POLE_LENGTH * scale;

    const poleAngleRad = poleAngleDeg * Math.PI / 180;

    const rearLeftX = centerX - trackWidth / 2;
    const rearRightX = centerX + trackWidth / 2;
    const rearY = centerY + wheelbase / 2;

    const frontLeftX = centerX - trackWidth / 2;
    const frontRightX = centerX + trackWidth / 2;
    const frontY = centerY - wheelbase / 2;

    const R = Math.abs(poleAngleRad) < 0.001 ? 999999 : CHARIOT_WHEELBASE / Math.tan(Math.abs(poleAngleRad));
    const innerAngle = Math.atan(CHARIOT_WHEELBASE / (R - CHARIOT_TRACK_WIDTH / 2));
    const outerAngle = Math.atan(CHARIOT_WHEELBASE / (R + CHARIOT_TRACK_WIDTH / 2));

    ctx.fillStyle = '#16213e';
    ctx.strokeStyle = '#e94560';
    ctx.lineWidth = 2;
    ctx.fillRect(centerX - trackWidth / 2 + 10, frontY + 20, trackWidth - 20, wheelbase - 40);
    ctx.strokeRect(centerX - trackWidth / 2 + 10, frontY + 20, trackWidth - 20, wheelbase - 40);

    const wheelRadius = 15;
    const wheelWidth = 8;

    drawWheel2D(ctx, rearLeftX, rearY, wheelRadius, wheelWidth, 0, '#2c1810');
    drawWheel2D(ctx, rearRightX, rearY, wheelRadius, wheelWidth, 0, '#2c1810');

    const leftWheelAngle = poleAngleDeg > 0 ? outerAngle : -innerAngle;
    const rightWheelAngle = poleAngleDeg > 0 ? innerAngle : -outerAngle;

    drawWheel2D(ctx, frontLeftX, frontY, wheelRadius, wheelWidth, leftWheelAngle, '#00d9ff');
    drawWheel2D(ctx, frontRightX, frontY, wheelRadius, wheelWidth, rightWheelAngle, '#00d9ff');

    ctx.strokeStyle = '#d4af37';
    ctx.lineWidth = 3;

    ctx.beginPath();
    ctx.moveTo(centerX, centerY - wheelbase / 2 + 30);
    const poleTipX = centerX + poleLen * Math.sin(poleAngleRad);
    const poleTipY = centerY - wheelbase / 2 + 30 - poleLen * Math.cos(poleAngleRad);
    ctx.lineTo(poleTipX, poleTipY);
    ctx.stroke();

    ctx.fillStyle = '#d4af37';
    ctx.beginPath();
    ctx.arc(poleTipX, poleTipY, 8, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = '#00ff88';
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 3]);

    const tieRodY = frontY + 25;
    const armLen = 25;

    const leftArmEndX = frontLeftX + armLen * Math.sin(leftWheelAngle);
    const leftArmEndY = tieRodY + armLen * Math.cos(leftWheelAngle);

    const rightArmEndX = frontRightX + armLen * Math.sin(rightWheelAngle);
    const rightArmEndY = tieRodY + armLen * Math.cos(rightWheelAngle);

    ctx.beginPath();
    ctx.moveTo(leftArmEndX, leftArmEndY);
    ctx.lineTo(rightArmEndX, rightArmEndY);
    ctx.stroke();

    ctx.setLineDash([]);

    ctx.fillStyle = '#aaa';
    ctx.font = '11px sans-serif';
    ctx.fillText('辕杆', poleTipX + 10, poleTipY - 5);
    ctx.fillText('转向横拉杆', (leftArmEndX + rightArmEndX) / 2 - 30, leftArmEndY - 8);

    ctx.fillStyle = '#fff';
    ctx.font = 'bold 12px sans-serif';
    ctx.fillText(`内轮转角: ${(Math.abs(innerAngle) * 180 / Math.PI).toFixed(1)}°`, 20, 30);
    ctx.fillText(`外轮转角: ${(Math.abs(outerAngle) * 180 / Math.PI).toFixed(1)}°`, 20, 50);
}


function drawWheel2D(ctx, x, y, radius, width, angle, color) {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(angle);

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.roundRect(-width / 2, -radius, width, radius * 2, 3);
    ctx.fill();

    ctx.strokeStyle = '#d4af37';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(-width / 2 - 2, 0);
    ctx.lineTo(width / 2 + 2, 0);
    ctx.stroke();

    ctx.restore();
}


function drawRolloverGauge(riskPercent) {
    const canvas = document.getElementById('rolloverGauge');
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    const centerX = w / 2;
    const centerY = h - 10;
    const radius = h * 0.9;

    ctx.strokeStyle = '#1a4a7a';
    ctx.lineWidth = 12;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.arc(centerX, centerY, radius, Math.PI, 2 * Math.PI);
    ctx.stroke();

    const gradient = ctx.createLinearGradient(0, centerY, w, centerY);
    gradient.addColorStop(0, '#00ff88');
    gradient.addColorStop(0.5, '#ffc107');
    gradient.addColorStop(1, '#e94560');

    ctx.strokeStyle = gradient;
    ctx.lineWidth = 12;
    ctx.beginPath();
    const angle = Math.PI + (riskPercent / 100) * Math.PI;
    ctx.arc(centerX, centerY, radius, Math.PI, angle);
    ctx.stroke();

    const needleAngle = Math.PI + (riskPercent / 100) * Math.PI;
    const needleLen = radius - 5;
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(
        centerX + Math.cos(needleAngle) * needleLen,
        centerY + Math.sin(needleAngle) * needleLen
    );
    ctx.stroke();

    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.arc(centerX, centerY, 6, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#fff';
    ctx.font = 'bold 18px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(`${riskPercent.toFixed(1)}%`, centerX, centerY + 18);
}


function updateSensorDisplay(data) {
    document.getElementById('sensorPoleAngle').textContent = `${data.pole_angle.toFixed(1)}°`;
    document.getElementById('sensorSlipRate').textContent = data.slip_rate.toFixed(3);
    document.getElementById('sensorRollAngle').textContent = `${data.roll_angle.toFixed(1)}°`;
    document.getElementById('sensorFriction').textContent = data.friction_coeff.toFixed(3);

    document.getElementById('barPoleAngle').style.width = `${((data.pole_angle + 40) / 80) * 100}%`;
    document.getElementById('barSlipRate').style.width = `${data.slip_rate * 100}%`;
    document.getElementById('barRollAngle').style.width = `${((data.roll_angle + 35) / 70) * 100}%`;
    document.getElementById('barFriction').style.width = `${data.friction_coeff * 100}%`;
}


function updateSteeringDisplay(analysis) {
    const r = analysis.turning_radius;
    document.getElementById('turningRadius').textContent =
        (r === Infinity || r > 999 || r == null) ? '∞ m' : `${r.toFixed(2)} m`;
    document.getElementById('innerWheelAngle').textContent = `${(analysis.inner_wheel_angle || 0).toFixed(1)}°`;
    document.getElementById('outerWheelAngle').textContent = `${(analysis.outer_wheel_angle || 0).toFixed(1)}°`;
    document.getElementById('wheelSpeedDiff').textContent = `${((analysis.wheel_speed_diff || 0) * 100).toFixed(2)}%`;
    document.getElementById('ackermannError').textContent = `${((analysis.ackermann_error || 0) * 100).toFixed(2)}%`;
}


function updateStabilityDisplay(analysis) {
    document.getElementById('yawRate').textContent = `${(analysis.yaw_rate || 0).toFixed(2)}°/s`;
    document.getElementById('lateralAccel').textContent = `${(analysis.lateral_acceleration || 0).toFixed(2)} m/s²`;
    document.getElementById('rollCenterHeight').textContent = `${(analysis.roll_center_height || 0).toFixed(3)} m`;
    document.getElementById('stabilityIndex').textContent = (analysis.stability_index || 0).toFixed(2);
    document.getElementById('criticalSpeed').textContent =
        analysis.critical_speed ? `${analysis.critical_speed.toFixed(2)} m/s` : '— m/s';

    drawRolloverGauge(analysis.rollover_risk || 0);

    if (chariotGroup) {
        const rollRad = (analysis.roll_angle || 0) * Math.PI / 180;
        chariotGroup.rotation.z = rollRad * 0.3;
    }
}


function addAlert(alert) {
    const alertsList = document.getElementById('alertsList');
    const noAlerts = alertsList.querySelector('.no-alerts');
    if (noAlerts) noAlerts.remove();

    const alertItem = document.createElement('div');
    alertItem.className = `alert-item ${alert.severity}`;

    const timeStr = new Date(alert.timestamp * 1000).toLocaleTimeString();

    alertItem.innerHTML = `
        <div class="alert-message">${alert.message}</div>
        <div class="alert-time">${timeStr} · ${alert.vehicle_id}</div>
    `;

    alertsList.insertBefore(alertItem, alertsList.firstChild);

    while (alertsList.children.length > 20) {
        alertsList.removeChild(alertsList.lastChild);
    }
}


async function fetchSteeringAnalysis(poleAngle, speed) {
    try {
        const response = await fetch(`${API_BASE}/api/analysis/steering`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                pole_angle: poleAngle,
                vehicle_speed: speed,
                friction_coeff: 0.7
            })
        });
        return await response.json();
    } catch (e) {
        console.error('获取转向分析失败:', e);
        return null;
    }
}


function connectWebSocket() {
    const wsProto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProto}//${window.location.host}/ws/realtime`;
    socket = new WebSocket(wsUrl);

    socket.onopen = () => {
        document.getElementById('connectionStatus').textContent = '已连接';
        document.getElementById('connectionStatus').classList.add('connected');
    };

    socket.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);

            if (data.sensor_data && data.sensor_data.vehicle_id === currentVehicleId) {
                updateSensorDisplay(data.sensor_data);

                if (data.steering_analysis) {
                    updateSteeringDisplay(data.steering_analysis);
                }

                if (data.stability_analysis) {
                    updateStabilityDisplay(data.stability_analysis);
                }

                if (data.alerts && data.alerts.length > 0) {
                    data.alerts.forEach(alert => addAlert(alert));
                }
            }
        } catch (e) {
            console.error('解析WebSocket消息失败:', e);
        }
    };

    socket.onclose = () => {
        document.getElementById('connectionStatus').textContent = '未连接';
        document.getElementById('connectionStatus').classList.remove('connected');
        setTimeout(connectWebSocket, 3000);
    };

    socket.onerror = () => {
        console.error('WebSocket错误');
    };
}


function setupEventListeners() {
    document.getElementById('poleAngleSlider').addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        document.getElementById('poleAngleValue').textContent = value.toFixed(1);
        drawLinkageDiagram(value);
    });

    document.getElementById('speedSlider').addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        document.getElementById('speedValue').textContent = value.toFixed(1);
    });

    document.getElementById('vehicleSelect').addEventListener('change', (e) => {
        currentVehicleId = e.target.value;
    });

    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            setView(btn.dataset.view);
        });
    });
}


async function init() {
    initThreeJS();
    drawLinkageDiagram(0);
    drawRolloverGauge(0);
    setupEventListeners();

    setTimeout(() => connectWebSocket(), 500);

    try {
        const steering = await fetchSteeringAnalysis(0, 5);
        if (steering) updateSteeringDisplay(steering);
    } catch (e) {
        console.log('使用默认数据');
    }
}


init();
