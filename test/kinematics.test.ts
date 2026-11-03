import assert from "node:assert/strict";
import { describe, it } from "node:test";

import { DEFAULT_ARM, type Arm, clampToLimits, forwardKinematics, jacobian, makeArm, withinLimits } from "../src/core/arm.js";
import { solveIK } from "../src/core/ik.js";
import { deg, dist, rng } from "../src/core/math.js";

const arm = DEFAULT_ARM;
const close = (a: readonly number[], b: readonly number[], tol: number): void =>
  a.forEach((x, i) => assert.ok(Math.abs(x - b[i]!) <= tol, `index ${i}: ${x} vs ${b[i]} (tol ${tol})`));

function randomConfig(r: () => number, shrink = 0.9): number[] {
  return arm.joints.map((j) => {
    const mid = (j.min + j.max) / 2;
    return mid + (r() - 0.5) * (j.max - j.min) * shrink;
  });
}

describe("forward kinematics", () => {
  it("points straight up at the zero pose", () => {
    const pose = forwardKinematics(arm, [0, 0, 0, 0, 0, 0]);
    close(pose.tool, [0, 0.12 + 0.16 + 0.42 + 0.2 + 0.2 + 0.07 + 0.1, 0], 1e-12);
    close(pose.toolDir, [0, 1, 0], 1e-12);
  });

  it("matches a hand-worked planar case", () => {
    // Shoulder at +90 deg about +Z tips the rest of the arm toward -X.
    const pose = forwardKinematics(arm, [0, deg(90), 0, 0, 0, 0]);
    const reach = 0.42 + 0.2 + 0.2 + 0.07 + 0.1;
    close(pose.tool, [-reach, 0.28, 0], 1e-12);
    // Then the elbow at -90 deg folds the forearm back up.
    const bent = forwardKinematics(arm, [0, deg(90), deg(-90), 0, 0, 0]);
    close(bent.tool, [-0.42, 0.28 + 0.2 + 0.2 + 0.07 + 0.1, 0], 1e-12);
  });

  it("base yaw rotates the whole arm about the vertical axis", () => {
    const a = forwardKinematics(arm, [0, deg(40), deg(30), 0, deg(20), 0]).tool;
    const b = forwardKinematics(arm, [deg(90), deg(40), deg(30), 0, deg(20), 0]).tool;
    // Rotating +90 deg about +Y maps (x, y, z) to (z, y, -x).
    close(b, [a[2], a[1], -a[0]], 1e-12);
  });

  it("keeps link lengths constant in any pose", () => {
    const r = rng(7);
    for (let n = 0; n < 50; n++) {
      const pts = forwardKinematics(arm, randomConfig(r)).frames.map((f) => f.p);
      arm.joints.slice(1).forEach((j, i) => {
        assert.ok(Math.abs(dist(pts[i]!, pts[i + 1]!) - Math.hypot(...j.offset)) < 1e-12);
      });
    }
  });

  it("jacobian matches finite differences", () => {
    const r = rng(3);
    const q = randomConfig(r);
    const J = jacobian(arm, forwardKinematics(arm, q));
    const h = 1e-6;
    q.forEach((_, i) => {
      const qp = [...q];
      const qm = [...q];
      qp[i]! += h;
      qm[i]! -= h;
      const p = forwardKinematics(arm, qp);
      const m = forwardKinematics(arm, qm);
      for (let k = 0; k < 3; k++) {
        assert.ok(Math.abs((p.tool[k]! - m.tool[k]!) / (2 * h) - J[k]![i]!) < 1e-6);
        assert.ok(Math.abs((p.toolDir[k]! - m.toolDir[k]!) / (2 * h) - J[3 + k]![i]!) < 1e-6);
      }
    });
  });

  it("rejects malformed arms and joint vectors", () => {
    assert.throws(() => forwardKinematics(arm, [0, 0, 0]));
    const bad: Arm = { ...arm, joints: [{ ...arm.joints[0]!, min: 1, max: -1 }, ...arm.joints.slice(1)] };
    assert.throws(() => makeArm(bad));
  });
});

describe("inverse kinematics", () => {
  it("reaches 200 random reachable targets to within 0.1 mm", () => {
    const r = rng(42);
    let iterations = 0;
    for (let n = 0; n < 200; n++) {
      const goal = forwardKinematics(arm, randomConfig(r)).tool;
      const res = solveIK(arm, [0, 0.3, 0.6, 0, 0.4, 0], { position: goal });
      assert.ok(res.converged, `target ${n} did not converge: error ${res.positionError}`);
      assert.ok(dist(forwardKinematics(arm, res.q).tool, goal) < 1e-4);
      assert.ok(withinLimits(arm, res.q));
      iterations += res.iterations;
    }
    assert.ok(iterations / 200 < 60, `average iterations ${iterations / 200}`);
  });

  it("solves position plus tool direction", () => {
    const r = rng(9);
    for (let n = 0; n < 100; n++) {
      const pose = forwardKinematics(arm, randomConfig(r, 0.8));
      const res = solveIK(arm, [0, 0.3, 0.6, 0, 0.4, 0], { position: pose.tool, direction: pose.toolDir });
      assert.ok(res.converged, `target ${n}: pos ${res.positionError}, angle ${res.angleError}`);
      assert.ok(res.angleError < 1e-3);
      assert.ok(withinLimits(arm, res.q));
    }
  });

  it("points the tool straight down at a table-height target", () => {
    const res = solveIK(arm, [0, 0.5, 1, 0, 0.5, 0], { position: [0.45, 0.15, 0.2], direction: [0, -1, 0] });
    assert.ok(res.converged);
    const pose = forwardKinematics(arm, res.q);
    close(pose.toolDir, [0, -1, 0], 2e-3);
  });

  it("warm starts from the current pose with small changes", () => {
    const q0 = [0.3, 0.5, 0.9, 0.1, 0.6, 0];
    const start = forwardKinematics(arm, q0).tool;
    const res = solveIK(arm, q0, { position: [start[0] + 0.01, start[1], start[2]] }, { restarts: 0 });
    assert.ok(res.converged);
    assert.ok(Math.max(...res.q.map((v, i) => Math.abs(v - q0[i]!))) < 0.1);
  });

  it("returns the closest in-limit pose for an unreachable target", () => {
    const res = solveIK(arm, [0, 0.2, 0.2, 0, 0.2, 0], { position: [3, 0.5, 0] });
    assert.equal(res.converged, false);
    assert.ok(withinLimits(arm, res.q));
    // Best effort is the arm stretched straight at the target from the
    // shoulder joint at (0, 0.28, 0); the shoulder-to-tip length is 0.99 m.
    const expected = Math.hypot(3, 0.5 - 0.28) - 0.99;
    assert.ok(Math.abs(res.positionError - expected) < 1e-3, `got ${res.positionError}, expected ${expected}`);
  });

  it("respects tight joint limits and reports which joints are pinned", () => {
    const tight = makeArm({
      ...arm,
      joints: arm.joints.map((j, i) => (i === 2 ? { ...j, min: deg(-20), max: deg(20) } : j)),
    });
    // This target needs a sharply bent elbow, which the limit forbids.
    const res = solveIK(tight, [0, 0.3, 0.1, 0, 0.2, 0], { position: [0.15, 0.35, 0] });
    assert.ok(withinLimits(tight, res.q));
    assert.ok(Math.abs(res.q[2]!) <= deg(20) + 1e-12);
    if (!res.converged) assert.ok(res.atLimit.length > 0);
  });

  it("clamps seeds that start outside the limits", () => {
    const q = clampToLimits(arm, [10, -10, 0, 0, 0, 0]);
    assert.ok(withinLimits(arm, q));
    const res = solveIK(arm, [10, -10, 0, 0, 0, 0], { position: [0.3, 0.6, 0.1] });
    assert.ok(withinLimits(arm, res.q));
    assert.ok(res.converged);
  });
});
