// Collision checks between the arm and box obstacles.
//
// Each link is treated as a capsule (a segment between two joints plus a
// radius) and each obstacle as an axis-aligned box. The link hits the box when
// the closest distance from the segment to the box is below the radius.
//
// For that distance I use a small trick instead of a case-by-case
// segment/box routine: distance from a point to a convex set is a convex
// function, and walking along a straight segment keeps it convex in the
// segment parameter t. So a ternary search over t in [0, 1] finds the exact
// minimum (to floating point) with no special cases.

import { type Arm, forwardKinematics, skeleton } from "./arm.js";
import { type Vec3, lerp, norm } from "./math.js";
import type { Trajectory } from "./trajectory.js";

export interface Box {
  name: string;
  min: Vec3;
  max: Vec3;
}

export function boxFromCenter(name: string, center: Vec3, size: Vec3): Box {
  const h: Vec3 = [size[0] / 2, size[1] / 2, size[2] / 2];
  return {
    name,
    min: [center[0] - h[0], center[1] - h[1], center[2] - h[2]],
    max: [center[0] + h[0], center[1] + h[1], center[2] + h[2]],
  };
}

export function pointBoxDistance(p: Vec3, box: Box): number {
  const d: Vec3 = [
    Math.max(box.min[0] - p[0], 0, p[0] - box.max[0]),
    Math.max(box.min[1] - p[1], 0, p[1] - box.max[1]),
    Math.max(box.min[2] - p[2], 0, p[2] - box.max[2]),
  ];
  return norm(d);
}

export function segmentBoxDistance(a: Vec3, b: Vec3, box: Box, iterations = 80): number {
  let lo = 0;
  let hi = 1;
  for (let i = 0; i < iterations; i++) {
    const m1 = lo + (hi - lo) / 3;
    const m2 = hi - (hi - lo) / 3;
    if (pointBoxDistance(lerp(a, b, m1), box) <= pointBoxDistance(lerp(a, b, m2), box)) hi = m2;
    else lo = m1;
  }
  return Math.min(pointBoxDistance(lerp(a, b, (lo + hi) / 2), box), pointBoxDistance(a, box), pointBoxDistance(b, box));
}

export interface Contact {
  /** Link index: link i runs from skeleton point i to i + 1. */
  link: number;
  linkName: string;
  box: string;
  /** Distance between capsule surface and box (negative = penetration depth). */
  clearance: number;
}

export function linkNames(arm: Arm): string[] {
  return [...arm.joints.map((j) => `to ${j.name}`), "tool"];
}

/**
 * Check every link against every box. Link 0 (base to first joint) is skipped
 * because it is bolted to the table.
 */
export function checkCollisions(arm: Arm, q: readonly number[], boxes: readonly Box[], margin = 0): Contact[] {
  const pts = skeleton(arm, forwardKinematics(arm, q));
  const radii = [...arm.joints.map((j) => j.linkRadius), arm.toolRadius];
  const names = linkNames(arm);
  const contacts: Contact[] = [];
  for (let i = 1; i + 1 < pts.length; i++) {
    for (const box of boxes) {
      const clearance = segmentBoxDistance(pts[i]!, pts[i + 1]!, box) - radii[i]! - margin;
      if (clearance < 0) contacts.push({ link: i, linkName: names[i]!, box: box.name, clearance });
    }
  }
  return contacts;
}

export interface PathCheck {
  ok: boolean;
  /** First time (seconds) the arm touches a box, if it does. */
  time?: number;
  contacts: Contact[];
}

/** Sample a trajectory every `dt` seconds and report the first collision. */
export function checkTrajectory(arm: Arm, traj: Trajectory, boxes: readonly Box[], dt = 0.01): PathCheck {
  for (let t = 0; t <= traj.duration + 1e-9; t += dt) {
    const contacts = checkCollisions(arm, traj.sample(t).q, boxes);
    if (contacts.length) return { ok: false, time: t, contacts };
  }
  const end = checkCollisions(arm, traj.sample(traj.duration).q, boxes);
  return end.length ? { ok: false, time: traj.duration, contacts: end } : { ok: true, contacts: [] };
}
