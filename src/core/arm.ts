// Arm description and forward kinematics.
//
// An arm is a chain of revolute joints. Each joint sits at a fixed `offset`
// from the previous joint (expressed in the previous joint's frame) and
// rotates about its own `axis`. Walking the chain and composing
// translate-then-rotate gives every joint frame and the tool pose.

import { IDENTITY, type Mat3, type Vec3, add, axisAngle, cross, deg, mulMat, mulVec, normalize, sub } from "./math.js";

export interface Joint {
  name: string;
  /** Rotation axis in the joint's local frame (normalized on load). */
  axis: Vec3;
  /** Position of this joint relative to the previous joint frame. */
  offset: Vec3;
  /** Joint limits in radians. */
  min: number;
  max: number;
  /** Radius of the link that ends at this joint, used for collision checks. */
  linkRadius: number;
}

export interface Arm {
  name: string;
  base: Vec3;
  joints: Joint[];
  /** Tool tip offset from the last joint frame. */
  tool: Vec3;
  toolRadius: number;
}

export interface Frame {
  /** World position of the joint (or tool tip). */
  p: Vec3;
  /** World rotation after the joint has rotated. */
  R: Mat3;
  /** World direction of the joint axis. */
  axis: Vec3;
}

export interface Pose {
  frames: Frame[];
  tool: Vec3;
  /** Direction the tool is pointing (the tool offset direction, in world space). */
  toolDir: Vec3;
  toolR: Mat3;
}

export function makeArm(spec: Arm): Arm {
  if (spec.joints.length < 3 || spec.joints.length > 7) {
    throw new Error(`expected 3 to 7 joints, got ${spec.joints.length}`);
  }
  for (const j of spec.joints) {
    if (!(j.min < j.max)) throw new Error(`joint ${j.name}: min must be below max`);
  }
  return { ...spec, joints: spec.joints.map((j) => ({ ...j, axis: normalize(j.axis) })) };
}

export function forwardKinematics(arm: Arm, q: readonly number[]): Pose {
  if (q.length !== arm.joints.length) {
    throw new Error(`expected ${arm.joints.length} joint values, got ${q.length}`);
  }
  let R: Mat3 = IDENTITY;
  let p: Vec3 = arm.base;
  const frames: Frame[] = [];
  arm.joints.forEach((joint, i) => {
    p = add(p, mulVec(R, joint.offset));
    const axis = mulVec(R, joint.axis);
    R = mulMat(R, axisAngle(joint.axis, q[i]!));
    frames.push({ p, R, axis });
  });
  const tool = add(p, mulVec(R, arm.tool));
  return { frames, tool, toolDir: normalize(mulVec(R, arm.tool)), toolR: R };
}

/**
 * Geometric Jacobian of the tool tip, 6 x n. Rows 0-2 are linear velocity of
 * the tip; rows 3-5 are the rate of change of the tool direction. For a
 * revolute joint with world axis a at position p, the tip moves at
 * a x (tip - p) and the tool direction d changes at a x d.
 */
export function jacobian(arm: Arm, pose: Pose): number[][] {
  const rows: number[][] = Array.from({ length: 6 }, () => new Array<number>(arm.joints.length).fill(0));
  pose.frames.forEach((f, i) => {
    const lin = cross(f.axis, sub(pose.tool, f.p));
    const ang = cross(f.axis, pose.toolDir);
    for (let k = 0; k < 3; k++) {
      rows[k]![i] = lin[k]!;
      rows[3 + k]![i] = ang[k]!;
    }
  });
  return rows;
}

export function withinLimits(arm: Arm, q: readonly number[], eps = 1e-9): boolean {
  return arm.joints.every((j, i) => q[i]! >= j.min - eps && q[i]! <= j.max + eps);
}

export function clampToLimits(arm: Arm, q: readonly number[]): number[] {
  return arm.joints.map((j, i) => Math.min(j.max, Math.max(j.min, q[i]!)));
}

/** Points along the arm used for drawing and collision: base, every joint, tool tip. */
export function skeleton(arm: Arm, pose: Pose): Vec3[] {
  return [arm.base, ...pose.frames.map((f) => f.p), pose.tool];
}

/**
 * A 6-DOF arm roughly the size of a desktop manipulator (units: meters, Y up).
 * Yaw at the base, shoulder and elbow pitch, forearm roll, wrist pitch and a
 * tool roll. At all zeros it points straight up.
 */
export const DEFAULT_ARM: Arm = makeArm({
  name: "armlab-6",
  base: [0, 0, 0],
  joints: [
    { name: "base yaw", axis: [0, 1, 0], offset: [0, 0.12, 0], min: deg(-170), max: deg(170), linkRadius: 0.07 },
    { name: "shoulder", axis: [0, 0, 1], offset: [0, 0.16, 0], min: deg(-105), max: deg(105), linkRadius: 0.06 },
    { name: "elbow", axis: [0, 0, 1], offset: [0, 0.42, 0], min: deg(-150), max: deg(150), linkRadius: 0.05 },
    { name: "forearm roll", axis: [0, 1, 0], offset: [0, 0.2, 0], min: deg(-180), max: deg(180), linkRadius: 0.045 },
    { name: "wrist pitch", axis: [0, 0, 1], offset: [0, 0.2, 0], min: deg(-120), max: deg(120), linkRadius: 0.04 },
    { name: "tool roll", axis: [0, 1, 0], offset: [0, 0.07, 0], min: deg(-180), max: deg(180), linkRadius: 0.035 },
  ],
  tool: [0, 0.1, 0],
  toolRadius: 0.025,
});
