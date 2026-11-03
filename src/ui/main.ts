// Browser front end. All the robotics lives in src/core; this file only draws
// the arm from forward kinematics, forwards target drags to the IK solver and
// wires up the panel.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";

import {
  type Box,
  type Contact,
  DEFAULT_ARM,
  type IKResult,
  type Trajectory,
  type Vec3,
  boxFromCenter,
  checkCollisions,
  checkTrajectory,
  forwardKinematics,
  planTrajectory,
  skeleton,
  solveIK,
} from "../core/index.js";

const arm = DEFAULT_ARM;
const LIMITS = { maxVelocity: 1.2, maxAcceleration: 3 };
const DOWN: Vec3 = [0, -1, 0];

const obstacles: Box[] = [
  boxFromCenter("crate", [0.55, 0.15, 0.3], [0.26, 0.3, 0.26]),
  boxFromCenter("pillar", [-0.4, 0.45, 0.4], [0.14, 0.9, 0.14]),
  boxFromCenter("bin", [0.3, 0.08, -0.5], [0.32, 0.16, 0.3]),
];

const state = {
  q: [0, 0.55, 1.1, 0, 0.9, 0] as number[],
  pointDown: false,
  waypoints: [] as number[][],
  trajectory: null as Trajectory | null,
  playStart: -1,
  lastIK: null as IKResult | null,
};

// ---------- scene ----------
const canvasHost = document.getElementById("viewport")!;
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
canvasHost.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xeef1f5);
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
camera.position.set(1.35, 1.25, 2.15);

const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(0.05, 0.4, 0);
orbit.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x8a94a6, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 2.2);
sun.position.set(2, 3.5, 1.5);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = sun.shadow.camera.bottom = -1.5;
sun.shadow.camera.right = sun.shadow.camera.top = 1.5;
scene.add(sun);

const floor = new THREE.Mesh(new THREE.PlaneGeometry(6, 6), new THREE.MeshStandardMaterial({ color: 0xe2e6ec, roughness: 1 }));
floor.rotation.x = -Math.PI / 2;
floor.receiveShadow = true;
scene.add(floor);
const grid = new THREE.GridHelper(4, 40, 0xb7bfcc, 0xcfd5de);
grid.position.y = 0.001;
scene.add(grid);

const v3 = (p: Vec3): THREE.Vector3 => new THREE.Vector3(p[0], p[1], p[2]);
const UP = new THREE.Vector3(0, 1, 0);

// ---------- obstacles ----------
const boxMeshes = obstacles.map((b) => {
  const size = v3([b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]]);
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(size.x, size.y, size.z),
    new THREE.MeshStandardMaterial({ color: 0x9aa8bb, roughness: 0.8, transparent: true, opacity: 0.85 }),
  );
  mesh.position.copy(v3(b.min).add(size.clone().multiplyScalar(0.5)));
  mesh.castShadow = mesh.receiveShadow = true;
  scene.add(mesh);
  return mesh;
});

// ---------- arm ----------
const linkMat = new THREE.MeshStandardMaterial({ color: 0xf4f5f7, roughness: 0.45, metalness: 0.1 });
const hitMat = new THREE.MeshStandardMaterial({ color: 0xe5484d, roughness: 0.5 });
const jointMat = new THREE.MeshStandardMaterial({ color: 0x3a6df0, roughness: 0.4, metalness: 0.2 });
const radii = [...arm.joints.map((j) => j.linkRadius), arm.toolRadius];

const links = radii.map((r) => {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, 1, 28), linkMat);
  m.castShadow = true;
  scene.add(m);
  return m;
});
const housings = arm.joints.map((j) => {
  const r = j.linkRadius * 1.3;
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, r * 2.1, 32), jointMat);
  m.castShadow = true;
  scene.add(m);
  return m;
});
const baseMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.13, 0.15, 0.04, 40), jointMat);
baseMesh.position.y = 0.02;
baseMesh.castShadow = true;
scene.add(baseMesh);
const tipMesh = new THREE.Mesh(new THREE.SphereGeometry(arm.toolRadius * 1.1, 20, 16), jointMat);
scene.add(tipMesh);

function drawArm(q: readonly number[], contacts: Contact[]): void {
  const pose = forwardKinematics(arm, q);
  const pts = skeleton(arm, pose).map(v3);
  const hit = new Set(contacts.map((c) => c.link));
  links.forEach((mesh, i) => {
    const a = pts[i]!;
    const b = pts[i + 1]!;
    const dir = b.clone().sub(a);
    const len = dir.length();
    mesh.visible = len > 1e-6;
    if (!mesh.visible) return;
    mesh.position.copy(a).add(b).multiplyScalar(0.5);
    mesh.quaternion.setFromUnitVectors(UP, dir.normalize());
    mesh.scale.set(1, len, 1);
    mesh.material = hit.has(i) ? hitMat : linkMat;
  });
  pose.frames.forEach((f, i) => {
    housings[i]!.position.copy(v3(f.p));
    housings[i]!.quaternion.setFromUnitVectors(UP, v3(f.axis).normalize());
  });
  tipMesh.position.copy(v3(pose.tool));
  const touched = new Set(contacts.map((c) => c.box));
  boxMeshes.forEach((m, i) => {
    (m.material as THREE.MeshStandardMaterial).color.set(touched.has(obstacles[i]!.name) ? 0xf2a0a2 : 0x9aa8bb);
  });
}

// ---------- target ----------
const target = new THREE.Mesh(
  new THREE.SphereGeometry(0.035, 24, 18),
  new THREE.MeshStandardMaterial({ color: 0xff8a00, emissive: 0x552200, roughness: 0.4 }),
);
target.position.copy(v3(forwardKinematics(arm, state.q).tool));
scene.add(target);

const gapLine = new THREE.Line(
  new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
  new THREE.LineDashedMaterial({ color: 0xff8a00, dashSize: 0.02, gapSize: 0.015 }),
);
scene.add(gapLine);

const gizmo = new TransformControls(camera, renderer.domElement);
gizmo.attach(target);
gizmo.setSize(0.8);
scene.add(gizmo.getHelper());
gizmo.addEventListener("dragging-changed", (e) => {
  orbit.enabled = !e.value;
});
gizmo.addEventListener("objectChange", () => {
  if (state.playStart < 0) solveToTarget();
});

// ---------- path preview ----------
const waypointGroup = new THREE.Group();
scene.add(waypointGroup);
const pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x3a6df0 }));
scene.add(pathLine);

// ---------- panel ----------
const $ = <T extends HTMLElement>(id: string): T => document.getElementById(id) as T;
const ikStatus = $("ik-status");
const collisionStatus = $("collision-status");
const pathStatus = $("path-status");
const jointsEl = $("joints");

const sliders = arm.joints.map((j, i) => {
  const row = document.createElement("label");
  row.className = "joint";
  row.innerHTML = `<span class="name">${j.name}</span><input type="range" step="0.1"><span class="val"></span>`;
  const input = row.querySelector("input")!;
  input.min = String((j.min * 180) / Math.PI);
  input.max = String((j.max * 180) / Math.PI);
  input.addEventListener("input", () => {
    state.q[i] = (Number(input.value) * Math.PI) / 180;
    target.position.copy(v3(forwardKinematics(arm, state.q).tool));
    state.lastIK = null;
    ikStatus.textContent = "Moved by hand (forward kinematics). Target snapped to the tool.";
    ikStatus.className = "status";
    refresh();
  });
  jointsEl.appendChild(row);
  return { input, val: row.querySelector<HTMLSpanElement>(".val")!, joint: j };
});

function solveToTarget(): IKResult {
  const goal: Vec3 = [target.position.x, target.position.y, target.position.z];
  const t = state.pointDown ? { position: goal, direction: DOWN } : { position: goal };
  // Warm start from the current pose so the arm moves smoothly while dragging.
  // Only jump to a different configuration if that actually reaches the target.
  let res = solveIK(arm, state.q, t, { restarts: 0 });
  if (!res.converged) {
    const global = solveIK(arm, state.q, t);
    if (global.converged) res = global;
  }
  state.q = res.q;
  state.lastIK = res;
  const pinned = res.atLimit.map((k) => arm.joints[k]!.name).join(", ");
  if (res.converged) {
    ikStatus.textContent = `Solved in ${res.iterations} iterations, ${(res.positionError * 1000).toFixed(3)} mm off`;
    ikStatus.className = "status ok";
  } else {
    ikStatus.textContent =
      `Can't reach: closest pose is ${(res.positionError * 1000).toFixed(0)} mm away` + (pinned ? ` (at limit: ${pinned})` : "");
    ikStatus.className = "status warn";
  }
  refresh();
  return res;
}

function refresh(): void {
  const contacts = checkCollisions(arm, state.q, obstacles);
  drawArm(state.q, contacts);
  sliders.forEach((s, i) => {
    const d = (state.q[i]! * 180) / Math.PI;
    s.input.value = String(d);
    s.val.textContent = `${d.toFixed(1)}\u00b0`;
    const nearLimit = state.q[i]! <= s.joint.min + 1e-3 || state.q[i]! >= s.joint.max - 1e-3;
    s.val.classList.toggle("limit", nearLimit);
  });
  collisionStatus.textContent = contacts.length
    ? `Collision: ${contacts.map((c) => `${c.linkName} hits ${c.box}`).join(", ")}`
    : "No collisions";
  collisionStatus.className = contacts.length ? "status bad" : "status ok";
  const tool = forwardKinematics(arm, state.q).tool;
  const showGap = state.lastIK !== null && !state.lastIK.converged;
  gapLine.visible = showGap;
  if (showGap) {
    gapLine.geometry.setFromPoints([v3(tool), target.position.clone()]);
    gapLine.computeLineDistances();
  }
}

function rebuildPath(): void {
  waypointGroup.clear();
  state.waypoints.forEach((q, i) => {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(0.018, 16, 12),
      new THREE.MeshBasicMaterial({ color: i === 0 ? 0x1f9d55 : 0x3a6df0 }),
    );
    m.position.copy(v3(forwardKinematics(arm, q).tool));
    waypointGroup.add(m);
  });
  state.trajectory = state.waypoints.length >= 2 ? planTrajectory(state.waypoints, LIMITS) : null;
  const traj = state.trajectory;
  if (!traj) {
    pathLine.visible = false;
    pathStatus.textContent = state.waypoints.length ? "1 waypoint. Move the target and add another." : "No waypoints yet.";
    pathStatus.className = "status";
    return;
  }
  const pts: THREE.Vector3[] = [];
  for (let t = 0; t <= traj.duration; t += traj.duration / 200) pts.push(v3(forwardKinematics(arm, traj.sample(t).q).tool));
  pathLine.geometry.setFromPoints(pts);
  pathLine.visible = true;
  const check = checkTrajectory(arm, traj, obstacles);
  const n = state.waypoints.length;
  pathStatus.textContent = check.ok
    ? `${n} waypoints, ${traj.duration.toFixed(1)} s, collision free`
    : `${n} waypoints, ${traj.duration.toFixed(1)} s. Collides at ${check.time!.toFixed(2)} s: ` +
      `${check.contacts[0]!.linkName} hits ${check.contacts[0]!.box}`;
  pathStatus.className = check.ok ? "status ok" : "status bad";
}

$("point-down").addEventListener("change", (e) => {
  state.pointDown = (e.target as HTMLInputElement).checked;
  solveToTarget();
});
$("add-waypoint").addEventListener("click", () => {
  state.waypoints.push([...state.q]);
  rebuildPath();
});
$("clear").addEventListener("click", () => {
  state.waypoints = [];
  state.playStart = -1;
  rebuildPath();
});
$("play").addEventListener("click", () => play());

function play(): boolean {
  if (!state.trajectory) return false;
  state.playStart = performance.now();
  gizmo.enabled = false;
  return true;
}

// ---------- loop ----------
function resize(): void {
  const { clientWidth: w, clientHeight: h } = canvasHost;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);

function frame(now: number): void {
  const traj = state.trajectory;
  if (state.playStart >= 0 && traj) {
    const t = (now - state.playStart) / 1000;
    state.q = traj.sample(t).q;
    target.position.copy(v3(forwardKinematics(arm, state.q).tool));
    state.lastIK = null;
    refresh();
    if (t >= traj.duration) {
      state.playStart = -1;
      gizmo.enabled = true;
    }
  }
  orbit.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}

resize();
solveToTarget();
rebuildPath();
requestAnimationFrame(frame);

// Small scripting hook, handy for poking at the solver from the dev console.
declare global {
  interface Window {
    armlab: unknown;
  }
}
window.armlab = {
  arm,
  obstacles,
  get q() {
    return [...state.q];
  },
  setTarget(x: number, y: number, z: number): IKResult {
    target.position.set(x, y, z);
    return solveToTarget();
  },
  addWaypoint(): number {
    state.waypoints.push([...state.q]);
    rebuildPath();
    return state.waypoints.length;
  },
  play,
  status: () => ({ ik: ikStatus.textContent, collision: collisionStatus.textContent, path: pathStatus.textContent }),
};
