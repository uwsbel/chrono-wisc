import argparse
import os
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree
import csv


def generate_open_racetrack(length_m=20.0, max_waypoints=20, seed=None):
    """
    Generate an open racetrack using cubic splines.

    Parameters:
        length_m (float): Target track length in meters.
        max_waypoints (int): Maximum number of control waypoints (2-20).
        seed (int | None): RNG seed for reproducibility.

    Returns:
        control_points (list[tuple[float, float]]): Control (x, y) waypoints.
        spline_points (list[tuple[float, float]]): Dense (x, y) spline points.
    """
    if max_waypoints < 2 or max_waypoints > 20:
        raise ValueError("max_waypoints must be between 2 and 20")
    if length_m <= 0:
        raise ValueError("length_m must be positive")

    rng = np.random.default_rng(seed)

    # Monotonic x keeps the track open and avoids self-intersections.
    x_ctrl = np.linspace(0.0, 1.0, max_waypoints)
    y_ctrl = rng.normal(0.0, 0.15, size=max_waypoints)

    t = np.linspace(0.0, 1.0, max_waypoints)
    cs_x = CubicSpline(t, x_ctrl, bc_type="natural")
    cs_y = CubicSpline(t, y_ctrl, bc_type="natural")

    t_dense = np.linspace(0.0, 1.0, 1000)
    x_smooth = cs_x(t_dense)
    y_smooth = cs_y(t_dense)

    dx = np.diff(x_smooth)
    dy = np.diff(y_smooth)
    current_length = np.sum(np.sqrt(dx**2 + dy**2))

    scale_factor = length_m / current_length
    x_ctrl *= scale_factor
    y_ctrl *= scale_factor
    x_smooth *= scale_factor
    y_smooth *= scale_factor

    control_points = list(zip(x_ctrl, y_ctrl))
    spline_points = list(zip(x_smooth, y_smooth))
    return control_points, spline_points


def plot_racetrack(waypoints, length_m, control_points=None):
    import matplotlib.pyplot as plt

    x, y = zip(*waypoints)

    plt.figure(figsize=(8, 6))
    plt.plot(x, y, "b-", lw=2)
    if control_points:
        cx, cy = zip(*control_points)
        plt.scatter(cx, cy, c="black", s=12, alpha=0.7)
    plt.scatter([x[0], x[-1]], [y[0], y[-1]], c=["green", "red"], s=60)

    plt.title(f"Open Racetrack (Length: {length_m:.1f} m)")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True, alpha=0.3)
    plt.axis("equal")
    plt.show()

def _load_centerline_csv(path):
    points = []
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
            except (ValueError, IndexError):
                continue
            points.append((x, y))
    if len(points) < 2:
        raise ValueError(f"Need at least 2 (x,y) points in CSV: {path}")
    return np.asarray(points, dtype=float)


def _resample_polyline(points_xy, ds):
    if ds <= 0:
        raise ValueError("ds must be positive")
    diffs = np.diff(points_xy, axis=0)
    seg_lens = np.sqrt(np.sum(diffs * diffs, axis=1))
    s = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total = s[-1]
    if total <= 0:
        raise ValueError("Degenerate polyline (zero length)")
    new_s = np.arange(0.0, total, ds, dtype=float)
    if new_s.size == 0 or new_s[-1] < total:
        new_s = np.concatenate([new_s, [total]])
    x = np.interp(new_s, s, points_xy[:, 0])
    y = np.interp(new_s, s, points_xy[:, 1])
    out = np.column_stack([x, y])
    out_diffs = np.diff(out, axis=0)
    out_seg_lens = np.sqrt(np.sum(out_diffs * out_diffs, axis=1))
    keep = np.concatenate([[True], out_seg_lens > 1e-12])
    return out[keep]


def _dist_sq_point_to_segment_batch(q_xy, a_xy, b_xy):
    ab = b_xy - a_xy
    aq = q_xy - a_xy
    denom = np.sum(ab * ab, axis=1)
    denom = np.where(denom > 0, denom, 1.0)
    t = np.sum(aq * ab, axis=1) / denom
    t = np.clip(t, 0.0, 1.0)
    proj = a_xy + ab * t[:, None]
    d = q_xy - proj
    return np.sum(d * d, axis=1)


def _min_dist_sq_to_polyline_approx(q_xy, poly_xy, tree):
    idx = tree.query(q_xy, k=1)[1]
    n = poly_xy.shape[0]
    d = np.full(q_xy.shape[0], np.inf, dtype=float)

    mask1 = idx > 0
    if np.any(mask1):
        i = idx[mask1]
        d[mask1] = _dist_sq_point_to_segment_batch(
            q_xy[mask1],
            poly_xy[i - 1],
            poly_xy[i],
        )

    mask2 = idx < (n - 1)
    if np.any(mask2):
        i = idx[mask2]
        d2 = _dist_sq_point_to_segment_batch(
            q_xy[mask2],
            poly_xy[i],
            poly_xy[i + 1],
        )
        d[mask2] = np.minimum(d[mask2], d2)

    return d


def generate_sph_and_bce_markers_from_centerline(
    centerline_xy,
    *,
    width,
    delta,
    depth,
    output_dir=".",
    num_bce=3,
    centerline_resolution=0.1,
):
    if width <= 0:
        raise ValueError("width must be positive")
    if delta <= 0:
        raise ValueError("delta must be positive")
    if depth <= 0:
        raise ValueError("depth must be positive")
    if num_bce <= 0:
        raise ValueError("num_bce must be positive")

    delta_mm = int(round(delta * 1000.0))
    if delta_mm <= 0:
        raise ValueError("delta is too small; delta*1000 must round to >= 1")

    os.makedirs(output_dir, exist_ok=True)
    particles_file = os.path.join(output_dir, f"particles_{delta_mm}mm.txt")
    bce_file = os.path.join(output_dir, f"bce_{delta_mm}mm.txt")

    poly_xy = _resample_polyline(np.asarray(centerline_xy, dtype=float), centerline_resolution)
    tree = cKDTree(poly_xy)

    r_inner = width / 2.0
    r_outer = r_inner + delta * num_bce
    r_inner_sq = r_inner * r_inner
    r_outer_sq = r_outer * r_outer

    xmin = float(np.min(poly_xy[:, 0]) - r_outer)
    xmax = float(np.max(poly_xy[:, 0]) + r_outer)
    ymin = float(np.min(poly_xy[:, 1]) - r_outer)
    ymax = float(np.max(poly_xy[:, 1]) + r_outer)

    x_vals = np.arange(xmin, xmax + 0.5 * delta, delta, dtype=float)
    y_vals = np.arange(ymin, ymax + 0.5 * delta, delta, dtype=float)

    particle_z = np.arange(-depth, 0.0 + 0.5 * delta, delta, dtype=float)
    bottom_bce_z = -depth - delta * np.arange(1, num_bce + 1, dtype=float)
    wall_z = np.arange(
        -depth - num_bce * delta,
        0.0 + num_bce * delta + 0.5 * delta,
        delta,
        dtype=float,
    )

    n_particles = 0
    n_bce = 0

    with open(particles_file, "w", newline="") as f_part, open(bce_file, "w", newline="") as f_bce:
        f_part.write("x, y, z,\n")
        f_bce.write("x, y, z,\n")

        row = np.empty((x_vals.size, 3), dtype=float)
        row[:, 0] = x_vals

        for y in y_vals:
            row[:, 1] = y
            d_sq = _min_dist_sq_to_polyline_approx(row[:, :2], poly_xy, tree)
            inner = d_sq <= r_inner_sq
            outer = d_sq <= r_outer_sq
            wall = outer & ~inner

            if np.any(inner):
                xs = row[inner, 0]
                arr = np.empty((xs.size, 3), dtype=float)
                arr[:, 0] = xs
                arr[:, 1] = y

                for z in particle_z:
                    arr[:, 2] = z
                    np.savetxt(f_part, arr, fmt="%.6f, %.6f, %.6f,")
                    n_particles += arr.shape[0]

                for z in bottom_bce_z:
                    arr[:, 2] = z
                    np.savetxt(f_bce, arr, fmt="%.6f, %.6f, %.6f,")
                    n_bce += arr.shape[0]

            if np.any(wall):
                xs = row[wall, 0]
                arr = np.empty((xs.size, 3), dtype=float)
                arr[:, 0] = xs
                arr[:, 1] = y

                for z in wall_z:
                    arr[:, 2] = z
                    np.savetxt(f_bce, arr, fmt="%.6f, %.6f, %.6f,")
                    n_bce += arr.shape[0]

    return {
        "particles_file": particles_file,
        "bce_file": bce_file,
        "n_particles": int(n_particles),
        "n_bce": int(n_bce),
        "delta_mm": int(delta_mm),
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate an open racetrack and (optionally) SPH/BCE marker files.",
        epilog=(
            "Examples:\n"
            "  # Generate a random open track (writes control points CSV)\n"
            "  python racetrack_gen.py --length 20 --max-waypoints 20 --seed 1 --output track.csv\n"
            "\n"
            "  # Generate SPH/BCE markers from a centerline CSV (x,y)\n"
            "  python racetrack_gen.py --input-waypoints track.csv --markers --width 4 --delta 0.02 "
            "--output-dir out --no-plot\n"
            "\n"
            "Notes:\n"
            "  - Current marker generation assumes a flat surface z=0.\n"
            "  - num_bce is fixed at 3 and depth defaults to 0.1m.\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--length",
        type=float,
        default=20.0,
        help="Target track length in meters (default: 20).",
    )
    parser.add_argument(
        "--max-waypoints",
        type=int,
        default=20,
        help="Maximum number of control waypoints (2-20, default: 20).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducibility.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="convex_racetrack.csv",
        help="Output CSV file path (default: convex_racetrack.csv).",
    )
    parser.add_argument(
        "--input-waypoints",
        type=str,
        default=None,
        help="Optional CSV with centerline (x,y) waypoints to generate markers from.",
    )
    parser.add_argument(
        "--markers",
        action="store_true",
        help="Generate `particles_*.txt` and `bce_*.txt` marker files.",
    )
    parser.add_argument(
        "--width",
        type=float,
        default=None,
        help="Road width in meters (required with --markers).",
    )
    parser.add_argument(
        "--delta",
        type=float,
        default=None,
        help="Marker spacing in meters (required with --markers).",
    )
    parser.add_argument(
        "--depth",
        type=float,
        default=0.1,
        help="Granular material depth below surface in meters (default: 0.1).",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=".",
        help="Directory for marker files (default: current directory).",
    )
    parser.add_argument(
        "--centerline-resolution",
        type=float,
        default=0.1,
        help="Resample spacing for centerline used in marker generation (default: 0.1m).",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable plotting.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    control_points = None
    spline_points = None
    if args.input_waypoints:
        centerline_xy = _load_centerline_csv(args.input_waypoints)
    else:
        control_points, spline_points = generate_open_racetrack(
            length_m=args.length,
            max_waypoints=args.max_waypoints,
            seed=args.seed,
        )
        with open(args.output, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(control_points)
        centerline_xy = np.asarray(spline_points, dtype=float)

    if args.markers:
        if args.width is None or args.delta is None:
            raise SystemExit("--markers requires --width and --delta")
        res = generate_sph_and_bce_markers_from_centerline(
            centerline_xy,
            width=float(args.width),
            delta=float(args.delta),
            depth=float(args.depth),
            num_bce=3,
            output_dir=args.output_dir,
            centerline_resolution=float(args.centerline_resolution),
        )
        print(
            f"Wrote {res['n_particles']} particles to: {res['particles_file']}\n"
            f"Wrote {res['n_bce']} BCE markers to: {res['bce_file']}"
        )

    if not args.no_plot and spline_points is not None:
        plot_racetrack(spline_points, args.length, control_points=control_points)


if __name__ == "__main__":
    main()
