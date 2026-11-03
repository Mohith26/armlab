// Joint-space trajectories through a list of waypoints.
//
// Each segment uses a quintic (minimum-jerk) time scaling
//     s(t) = 10 t^3 - 15 t^4 + 6 t^5,   t in [0, 1]
// which starts and ends with zero velocity and zero acceleration, so the arm
// eases in and out of every waypoint without jerks. All joints share the same
// segment duration, chosen as the shortest time that keeps every joint under
// its velocity and acceleration limits. For the quintic profile the peak
// velocity is 1.875 * delta / T and the peak acceleration is about
// 5.7735 * delta / T^2, which gives the duration directly.

const PEAK_VEL = 1.875;
const PEAK_ACC = 10 / Math.sqrt(3); // 5.7735...

export interface Limits {
  /** Max joint speed, rad/s (one value for all joints or one per joint). */
  maxVelocity: number | number[];
  /** Max joint acceleration, rad/s^2. */
  maxAcceleration: number | number[];
  /** Shortest allowed segment, seconds. */
  minDuration?: number;
}

export interface Segment {
  from: number[];
  to: number[];
  start: number;
  duration: number;
}

export interface Sample {
  q: number[];
  qd: number[];
  /** Index of the segment this sample is in. */
  segment: number;
}

export interface Trajectory {
  segments: Segment[];
  duration: number;
  sample(t: number): Sample;
}

const pick = (v: number | number[], i: number): number => (Array.isArray(v) ? v[i]! : v);

export function segmentDuration(from: readonly number[], to: readonly number[], limits: Limits): number {
  let T = limits.minDuration ?? 0.05;
  from.forEach((a, i) => {
    const delta = Math.abs(to[i]! - a);
    T = Math.max(T, (PEAK_VEL * delta) / pick(limits.maxVelocity, i), Math.sqrt((PEAK_ACC * delta) / pick(limits.maxAcceleration, i)));
  });
  return T;
}

export function quintic(t: number): { s: number; ds: number } {
  const u = Math.min(1, Math.max(0, t));
  return { s: u * u * u * (10 - 15 * u + 6 * u * u), ds: 30 * u * u * (1 - u) * (1 - u) };
}

export function planTrajectory(waypoints: readonly (readonly number[])[], limits: Limits): Trajectory {
  if (waypoints.length < 2) throw new Error("need at least two waypoints");
  const n = waypoints[0]!.length;
  if (waypoints.some((w) => w.length !== n)) throw new Error("waypoints have different joint counts");

  const segments: Segment[] = [];
  let start = 0;
  for (let k = 0; k + 1 < waypoints.length; k++) {
    const from = [...waypoints[k]!];
    const to = [...waypoints[k + 1]!];
    const duration = segmentDuration(from, to, limits);
    segments.push({ from, to, start, duration });
    start += duration;
  }

  const sample = (t: number): Sample => {
    const time = Math.min(Math.max(t, 0), start);
    let idx = segments.findIndex((s) => time < s.start + s.duration);
    if (idx === -1) idx = segments.length - 1;
    const seg = segments[idx]!;
    const { s, ds } = quintic((time - seg.start) / seg.duration);
    return {
      q: seg.from.map((a, i) => a + (seg.to[i]! - a) * s),
      qd: seg.from.map((a, i) => ((seg.to[i]! - a) * ds) / seg.duration),
      segment: idx,
    };
  };

  return { segments, duration: start, sample };
}
