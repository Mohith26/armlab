// IK success rate and speed on random reachable targets.
// Targets come from forward kinematics of random in-limit poses, so every one
// of them is reachable. Run after `npm run build`:  node scripts/bench.mjs
import { DEFAULT_ARM as arm, forwardKinematics } from "../dist/src/core/arm.js";
import { solveIK } from "../dist/src/core/ik.js";
import { rng } from "../dist/src/core/math.js";

const N = 2000;
for (const withDirection of [false, true]) {
  for (const restarts of [0, 24]) {
    const r = rng(2026);
    let converged = 0;
    const t0 = performance.now();
    for (let n = 0; n < N; n++) {
      const q = arm.joints.map((j) => j.min + (0.05 + 0.9 * r()) * (j.max - j.min));
      const pose = forwardKinematics(arm, q);
      const target = withDirection ? { position: pose.tool, direction: pose.toolDir } : { position: pose.tool };
      const res = solveIK(arm, [0, 0.3, 0.6, 0, 0.4, 0], target, { restarts });
      converged += res.converged ? 1 : 0;
    }
    const ms = (performance.now() - t0) / N;
    const label = withDirection ? "position + direction" : "position only       ";
    console.log(
      `${label}  restarts=${String(restarts).padStart(2)}  converged ${converged}/${N} (${((100 * converged) / N).toFixed(1)}%)  ` +
        `avg ${ms.toFixed(3)} ms/solve`,
    );
  }
}
