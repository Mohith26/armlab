import assert from "node:assert/strict";
import { describe, it } from "node:test";

import { planTrajectory, quintic, segmentDuration } from "../src/core/trajectory.js";

const limits = { maxVelocity: 1.5, maxAcceleration: 4 };

describe("trajectory", () => {
  it("quintic starts and ends at rest", () => {
    assert.deepEqual(quintic(0), { s: 0, ds: 0 });
    assert.deepEqual(quintic(1), { s: 1, ds: 0 });
    assert.equal(quintic(0.5).s, 0.5);
  });

  it("passes through every waypoint and stops there", () => {
    const wps = [
      [0, 0, 0],
      [1, -0.5, 0.2],
      [0.2, 0.3, -1],
    ];
    const traj = planTrajectory(wps, limits);
    assert.equal(traj.segments.length, 2);
    let t = 0;
    wps.forEach((w, k) => {
      const s = traj.sample(t);
      s.q.forEach((x, i) => assert.ok(Math.abs(x - w[i]!) < 1e-12));
      s.qd.forEach((v) => assert.ok(Math.abs(v) < 1e-12));
      if (k < traj.segments.length) t += traj.segments[k]!.duration;
    });
  });

  it("never exceeds the velocity limit and is continuous", () => {
    const traj = planTrajectory([[0, 0], [2, 0.1], [-1, 1]], limits);
    let peak = 0;
    let prev = traj.sample(0).q;
    for (let t = 0; t <= traj.duration; t += 0.001) {
      const s = traj.sample(t);
      peak = Math.max(peak, ...s.qd.map(Math.abs));
      s.q.forEach((x, i) => assert.ok(Math.abs(x - prev[i]!) < 0.01));
      prev = s.q;
    }
    assert.ok(peak <= 1.5 + 1e-9, `peak ${peak}`);
    assert.ok(peak > 1.4, "should use most of the allowed speed on the long move");
  });

  it("respects the acceleration limit on short moves", () => {
    const T = segmentDuration([0], [0.05], limits);
    assert.ok(Math.abs(T - Math.sqrt((10 / Math.sqrt(3)) * 0.05 / 4)) < 1e-12);
  });

  it("synchronizes joints so they start and finish together", () => {
    const traj = planTrajectory([[0, 0], [1, 0.1]], limits);
    const mid = traj.sample(traj.duration / 2).q;
    assert.ok(Math.abs(mid[0]! - 0.5) < 1e-12 && Math.abs(mid[1]! - 0.05) < 1e-12);
  });

  it("clamps sample times and rejects bad input", () => {
    const traj = planTrajectory([[0], [1]], limits);
    assert.equal(traj.sample(-5).q[0], 0);
    assert.equal(traj.sample(1e9).q[0], 1);
    assert.throws(() => planTrajectory([[0]], limits));
    assert.throws(() => planTrajectory([[0], [1, 2]], limits));
  });
});
