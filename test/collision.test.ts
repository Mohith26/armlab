import assert from "node:assert/strict";
import { describe, it } from "node:test";

import { DEFAULT_ARM, forwardKinematics } from "../src/core/arm.js";
import { boxFromCenter, checkCollisions, checkTrajectory, pointBoxDistance, segmentBoxDistance } from "../src/core/collision.js";
import { type Vec3, deg, lerp, rng } from "../src/core/math.js";
import { planTrajectory } from "../src/core/trajectory.js";

const unit = boxFromCenter("unit", [0, 0, 0], [2, 2, 2]);

describe("distance", () => {
  it("point to box", () => {
    assert.equal(pointBoxDistance([0.5, 0.5, 0.5], unit), 0);
    assert.equal(pointBoxDistance([3, 0, 0], unit), 2);
    assert.ok(Math.abs(pointBoxDistance([2, 2, 1], unit) - Math.SQRT2) < 1e-12);
  });

  it("segment through a box is zero, parallel segment is exact", () => {
    assert.ok(segmentBoxDistance([-5, 0, 0], [5, 0, 0], unit) < 1e-9);
    assert.ok(Math.abs(segmentBoxDistance([-5, 3, 0], [5, 3, 0], unit) - 2) < 1e-9);
  });

  it("matches brute force on random segments", () => {
    const r = rng(11);
    const rp = (): Vec3 => [r() * 6 - 3, r() * 6 - 3, r() * 6 - 3];
    for (let n = 0; n < 300; n++) {
      const a = rp();
      const b = rp();
      let brute = Infinity;
      for (let i = 0; i <= 20000; i++) brute = Math.min(brute, pointBoxDistance(lerp(a, b, i / 20000), unit));
      assert.ok(Math.abs(segmentBoxDistance(a, b, unit) - brute) < 1e-3);
      assert.ok(segmentBoxDistance(a, b, unit) <= brute + 1e-12);
    }
  });
});

describe("arm collisions", () => {
  const arm = DEFAULT_ARM;

  it("upright arm clears a box off to the side", () => {
    const box = boxFromCenter("crate", [0.5, 0.15, 0], [0.2, 0.3, 0.2]);
    assert.deepEqual(checkCollisions(arm, [0, 0, 0, 0, 0, 0], [box]), []);
  });

  it("reaching down into a box is detected on the right link", () => {
    const q = [0, deg(-80), deg(-40), 0, 0, 0];
    const tip = forwardKinematics(arm, q).tool;
    const box = boxFromCenter("crate", tip, [0.1, 0.1, 0.1]);
    const contacts = checkCollisions(arm, q, [box]);
    assert.ok(contacts.some((c) => c.linkName === "tool" && c.box === "crate"));
  });

  it("finds the first collision along a trajectory", () => {
    const box = boxFromCenter("wall", [-0.6, 0.6, 0], [0.1, 1.2, 1.2]);
    const traj = planTrajectory([[0, 0, 0, 0, 0, 0], [0, deg(90), 0, 0, 0, 0]], { maxVelocity: 1, maxAcceleration: 3 });
    const res = checkTrajectory(arm, traj, [box]);
    assert.equal(res.ok, false);
    assert.ok(res.time! > 0 && res.time! < traj.duration);
    const clear = checkTrajectory(arm, traj, [boxFromCenter("far", [2, 0.2, 2], [0.2, 0.2, 0.2])]);
    assert.equal(clear.ok, true);
  });
});
