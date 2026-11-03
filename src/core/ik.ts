// Inverse kinematics with damped least squares.
//
// Each iteration linearizes the arm around the current joint angles
// (the Jacobian J) and solves for a joint step dq that reduces the task error e:
//
//     dq = J^T (J J^T + lambda^2 I)^-1 e
//
// The damping term lambda keeps the step finite near singularities (arm fully
// stretched, wrist axes lined up), where a plain pseudo-inverse would ask for
// huge joint velocities. Lambda adapts like Levenberg-Marquardt: shrink it
// after a step that helped, grow it after one that did not.
//
// Joint limits use an active set: a joint sitting on a limit whose step would
// push it further out is frozen (its Jacobian column zeroed) and the step is
// re-solved, so the other joints pick up the slack instead of the solver
// fighting the clamp.

import { type Arm, type Pose, forwardKinematics, jacobian } from "./arm.js";
import { type Vec3, clamp, dot, norm, normalize, rng, solve, sub } from "./math.js";

export interface IKTarget {
  position: Vec3;
  /** Optional direction the tool should point, e.g. [0, -1, 0] for straight down. */
  direction?: Vec3;
}

export interface IKOptions {
  maxIterations: number;
  /** Converged when the tip is within this many meters of the target. */
  positionTolerance: number;
  /** ...and the tool direction is within this many radians (when constrained). */
  angleTolerance: number;
  /** How much a radian of direction error counts relative to a meter of position error. */
  directionWeight: number;
  initialDamping: number;
  /** Largest change for any joint in one iteration, in radians. */
  maxStep: number;
  /** Extra attempts from random in-limit seeds when the first attempt stalls. */
  restarts: number;
  seed: number;
}

export interface IKResult {
  q: number[];
  converged: boolean;
  iterations: number;
  positionError: number;
  angleError: number;
  /** Joints that ended up on a limit. */
  atLimit: number[];
}

export const DEFAULT_IK: IKOptions = {
  maxIterations: 150,
  positionTolerance: 1e-4,
  angleTolerance: 1e-3,
  directionWeight: 0.3,
  initialDamping: 0.05,
  maxStep: 0.25,
  restarts: 24,
  seed: 1,
};

interface Err {
  e: number[];
  cost: number;
  pos: number;
  ang: number;
}

function taskError(pose: Pose, target: IKTarget, w: number): Err {
  const dp = sub(target.position, pose.tool);
  const e = [dp[0], dp[1], dp[2]];
  let ang = 0;
  if (target.direction) {
    const d = target.direction;
    const dd = sub(d, pose.toolDir);
    e.push(w * dd[0], w * dd[1], w * dd[2]);
    ang = Math.acos(clamp(dot(d, pose.toolDir), -1, 1));
  }
  return { e, cost: e.reduce((s, x) => s + x * x, 0), pos: norm(dp), ang };
}

function dlsStep(J: number[][], e: number[], lambda: number, frozen: boolean[]): number[] {
  const m = e.length;
  const n = frozen.length;
  // A = J J^T + lambda^2 I, using only unfrozen columns.
  const A: number[][] = Array.from({ length: m }, (_, r) =>
    Array.from({ length: m }, (_, c) => {
      let s = r === c ? lambda * lambda : 0;
      for (let k = 0; k < n; k++) if (!frozen[k]) s += J[r]![k]! * J[c]![k]!;
      return s;
    }),
  );
  const y = solve(A, e);
  return Array.from({ length: n }, (_, k) => {
    if (frozen[k]) return 0;
    let s = 0;
    for (let r = 0; r < m; r++) s += J[r]![k]! * y[r]!;
    return s;
  });
}

function attempt(arm: Arm, q0: readonly number[], target: IKTarget, o: IKOptions): IKResult {
  const rows = target.direction ? 6 : 3;
  let q = q0.map((v, i) => clamp(v, arm.joints[i]!.min, arm.joints[i]!.max));
  let pose = forwardKinematics(arm, q);
  let err = taskError(pose, target, o.directionWeight);
  let lambda = o.initialDamping;
  let it = 0;
  const done = (x: Err): boolean =>
    x.pos <= o.positionTolerance && (!target.direction || x.ang <= o.angleTolerance);

  for (; it < o.maxIterations && !done(err); it++) {
    const J = jacobian(arm, pose).slice(0, rows);
    const frozen = arm.joints.map(() => false);
    let dq = dlsStep(J, err.e, lambda, frozen);
    // Active set: freeze joints pinned at a limit and pushing outward, then re-solve.
    for (let pass = 0; pass < arm.joints.length; pass++) {
      let changed = false;
      arm.joints.forEach((j, k) => {
        if (frozen[k]) return;
        const pushingOut = (q[k]! <= j.min + 1e-9 && dq[k]! < 0) || (q[k]! >= j.max - 1e-9 && dq[k]! > 0);
        if (pushingOut) {
          frozen[k] = true;
          changed = true;
        }
      });
      if (!changed || frozen.every(Boolean)) break;
      dq = dlsStep(J, err.e, lambda, frozen);
    }
    const biggest = Math.max(...dq.map(Math.abs));
    if (biggest > o.maxStep) dq = dq.map((v) => (v * o.maxStep) / biggest);

    const qNext = q.map((v, k) => clamp(v + dq[k]!, arm.joints[k]!.min, arm.joints[k]!.max));
    const poseNext = forwardKinematics(arm, qNext);
    const errNext = taskError(poseNext, target, o.directionWeight);
    if (errNext.cost < err.cost) {
      q = qNext;
      pose = poseNext;
      err = errNext;
      lambda = Math.max(lambda * 0.5, 1e-4);
    } else {
      lambda *= 4;
      if (lambda > 10) break; // no step in any direction helps: stuck on limits or out of reach
    }
  }
  const atLimit = arm.joints.flatMap((j, k) => (q[k]! <= j.min + 1e-6 || q[k]! >= j.max - 1e-6 ? [k] : []));
  return { q, converged: done(err), iterations: it, positionError: err.pos, angleError: err.ang, atLimit };
}

/**
 * Solve for joint angles that put the tool tip at `target.position` (and, if
 * given, point it along `target.direction`). Starts from `q0`, which makes
 * repeated calls while dragging a target smooth. If that attempt stalls in a
 * local minimum it retries from seeded random configurations and returns the
 * best result found. The returned `q` always respects joint limits.
 */
export function solveIK(arm: Arm, q0: readonly number[], target: IKTarget, options: Partial<IKOptions> = {}): IKResult {
  const o = { ...DEFAULT_IK, ...options };
  const t: IKTarget = target.direction ? { ...target, direction: normalize(target.direction) } : target;
  let best = attempt(arm, q0, t, o);
  const rand = rng(o.seed);
  for (let r = 0; r < o.restarts && !best.converged; r++) {
    const seed = arm.joints.map((j) => j.min + rand() * (j.max - j.min));
    const res = attempt(arm, seed, t, o);
    const score = (x: IKResult): number => x.positionError + o.directionWeight * x.angleError;
    if (res.converged || score(res) < score(best)) best = res;
  }
  return best;
}
