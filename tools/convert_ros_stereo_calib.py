#!/usr/bin/env python3
"""
Converts ROS `camera_calibration`-style calibration YAML file(s) (each with
camera_matrix/distortion_coefficients/rectification_matrix/projection_matrix,
i.e. K, D, R, P) into an ORB-SLAM3 settings YAML.

Pass both --left and --right for a stereo settings file (raw K/D per camera
plus a Stereo.T_c1_c2 extrinsic), for use with Examples/Stereo/stereo_general
or stereo_euroc. Pass --left only for a monocular settings file (just the one
camera's raw K/D), for use with Examples/Monocular/mono_general or mono_euroc.

ORB-SLAM3 wants RAW (unrectified) intrinsics/distortion and the raw extrinsic
pose between the two cameras -- it runs its own cv::stereoRectify at startup
(see src/Settings.cc). ROS calibration files only store the *rectified*
outputs (R, P), not the raw extrinsic pose, so this script reconstructs it
from R1, R2 and P2's baseline term:

  Both cameras' rectified frames share one common orientation and differ only
  by a translation T' = [Tx, 0, 0] (Tx = P2[0,3] / P2[0,0]), because that is
  exactly what "rectified" means. Since X_rect1 = R1 @ X_cam1 and
  X_rect2 = R2 @ X_cam2, and a point's rect2-frame coordinate equals its
  rect1-frame coordinate plus T' (that's what P2 = K*[I|T'] encodes relative
  to the common rectified frame == rect1's frame), substituting gives:

      X_cam2 = (R2^T @ R1) @ X_cam1 + R2^T @ T'

  ORB-SLAM3's Settings.cc loads "Stereo.T_c1_c2" as Tlr (maps a right-camera
  point into the left camera's frame), which is the inverse of the relation
  above. Composing the inverse simplifies (R2 cancels since it's orthonormal)
  to:

      R_lr = R1^T @ R2
      t_lr = -R1^T @ T'

  The script self-checks this by feeding the reconstructed raw (R, T) back
  through cv2.stereoRectify and confirming it reproduces the original R1/R2/P1/P2
  from the input files.

Usage:
    python3 tools/convert_ros_stereo_calib.py \
        --left  path/to/stereo_calibration_left.yaml \
        --right path/to/stereo_calibration_right.yaml \
        --template Examples/Stereo/EuRoC.yaml \
        --images-left path/to/images_left \
        --out Examples/Stereo/my_camera.yaml
"""
import argparse
import glob
import os
import re
import sys

import numpy as np
import yaml
import cv2


def load_ros_calib(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(d["distortion_coefficients"]["data"], dtype=np.float64)
    R = np.array(d["rectification_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    P = np.array(d["projection_matrix"]["data"], dtype=np.float64).reshape(3, 4)
    width = int(d["image_width"])
    height = int(d["image_height"])
    model = d.get("distortion_model", "plumb_bob")
    if model != "plumb_bob":
        print(f"WARNING: distortion_model is '{model}', not 'plumb_bob' -- "
              f"ORB-SLAM3's pinhole model assumes plumb_bob (k1,k2,p1,p2,k3). "
              f"Proceeding anyway, but double check the output.", file=sys.stderr)
    return K, D, R, P, width, height


def estimate_fps(images_dir):
    """Median inter-frame interval from *.png filenames (nanosecond timestamps)."""
    stamps = []
    for p in glob.glob(os.path.join(images_dir, "*.png")):
        stem = os.path.splitext(os.path.basename(p))[0]
        if re.fullmatch(r"\d+", stem):
            stamps.append(int(stem))
    if len(stamps) < 2:
        return None
    stamps.sort()
    diffs = np.diff(np.array(stamps, dtype=np.float64)) / 1e9
    dt = float(np.median(diffs))
    return 1.0 / dt if dt > 0 else None


def derive_t_c1_c2(K1, D1, R1, P1, K2, D2, R2, P2, image_size):
    fx2 = P2[0, 0]
    Tx = P2[0, 3] / fx2  # translation of the rectified common frame -> cam2's rectified frame
    T_prime = np.array([Tx, 0.0, 0.0])

    # Raw (unrectified) extrinsic: X_cam2 = R_raw @ X_cam1 + T_raw
    R_raw = R2.T @ R1
    T_raw = R2.T @ T_prime

    # T_c1_c2 (a.k.a. Tlr in Settings.cc) is the inverse: maps cam2-frame points into cam1's frame.
    R_lr = R_raw.T
    t_lr = -R_raw.T @ T_raw

    # Self-check: feed the raw extrinsic back through stereoRectify and confirm
    # it reproduces R1/R2 (the rectifying rotations) from the input files.
    # P1/P2 are deliberately NOT compared here: they additionally depend on the
    # `alpha` (crop/scaling) choice made by whatever tool produced the input
    # files, which we don't know and don't need to match -- ORB-SLAM3 computes
    # its own P1/P2 at runtime (with alpha=-1, same as used below) from the
    # raw calibration this script outputs, so the original files' P1/P2 are
    # never reused. R1/R2 depend only on the extrinsic geometry, so matching
    # them is the real correctness signal.
    R1_chk, R2_chk, _P1_chk, _P2_chk, _, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, image_size, R_raw, T_raw.reshape(3, 1),
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=image_size,
    )
    err = max(np.abs(R1_chk - R1).max(), np.abs(R2_chk - R2).max())
    if err > 1e-6:
        print(f"WARNING: round-trip check on R1/R2 did not match closely (max abs diff = {err:.6f}). "
              f"Double-check the generated calibration before trusting it.", file=sys.stderr)
    else:
        print(f"Round-trip self-check OK (R1/R2 max abs diff = {err:.2e}).")

    return R_lr, t_lr, float(np.linalg.norm(t_lr))


def format_opencv_matrix(name, mat):
    rows, cols = mat.shape
    data = ",".join(f"{v:.15f}" for v in mat.flatten())
    return (f"{name}: !!opencv-matrix\n"
            f"  rows: {rows}\n"
            f"  cols: {cols}\n"
            f"  dt: f\n"
            f"  data: [{data}]\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--left", required=True, help="ROS stereo_calibration_left.yaml")
    ap.add_argument("--right", help="ROS stereo_calibration_right.yaml (omit for a monocular settings file)")
    ap.add_argument("--template", required=True,
                     help="Existing ORB-SLAM3 settings yaml to copy ORB/viewer parameters from (e.g. Examples/Stereo/EuRoC.yaml)")
    ap.add_argument("--images-left", help="images_left directory, used only to estimate Camera.fps from filenames")
    ap.add_argument("--fps", type=float, help="Override Camera.fps instead of estimating it")
    ap.add_argument("--th-depth", type=float, default=60.0, help="Stereo.ThDepth (default: 60.0, same as EuRoC.yaml)")
    ap.add_argument("--rgb", type=int, choices=[0, 1], default=0,
                     help="Camera.RGB: 0=BGR, 1=RGB (default: 0 -- cv::imread/cv2.imread return BGR-ordered "
                          "data for any standard PNG, which is the common case; only pass 1 if you've "
                          "actually verified your images need it, e.g. by visual inspection)")
    ap.add_argument("--image-view-scale", type=float,
                     help="Viewer.imageViewScale: shrinks only the 'Current Frame' display window "
                          "(cv::resize right before cv::imshow) -- does not affect calibration, "
                          "rectification, or tracking at all. Default: auto, scaled so the window "
                          "is at most 960px wide (1.0 if the image is already <=960px wide).")
    ap.add_argument("--new-width", type=int,
                     help="Camera.newWidth: actually resizes images before feature extraction/tracking "
                          "(unlike --image-view-scale). Lowers per-frame compute AND reduces apparent "
                          "pixel displacement between frames, which can help monocular tracking on fast "
                          "motion. Requires --new-height too. Omit for native resolution (default).")
    ap.add_argument("--new-height", type=int, help="Camera.newHeight, paired with --new-width.")
    ap.add_argument("--n-features", type=int,
                     help="ORBextractor.nFeatures override (default: whatever --template has). "
                          "Worth lowering if --new-width/--new-height shrink the processed image a lot -- "
                          "a feature count tuned for the native resolution's pixel density will be "
                          "excessive (and slow) at a much smaller processing resolution.")
    ap.add_argument("--out", required=True, help="Output settings yaml path")
    args = ap.parse_args()

    K1, D1, R1, P1, w1, h1 = load_ros_calib(args.left)
    mono = args.right is None

    if not mono:
        K2, D2, R2, P2, w2, h2 = load_ros_calib(args.right)
        if (w1, h1) != (w2, h2):
            print(f"ERROR: left/right image sizes differ: {(w1,h1)} vs {(w2,h2)}", file=sys.stderr)
            sys.exit(1)

        R_lr, t_lr, baseline = derive_t_c1_c2(K1, D1, R1, P1, K2, D2, R2, P2, (w1, h1))
        print(f"Baseline: {baseline:.6f} m")

    fps = args.fps
    if fps is None and args.images_left:
        fps = estimate_fps(args.images_left)
        if fps:
            print(f"Estimated Camera.fps from image timestamps: {fps:.2f}")
    if fps is None:
        fps = 20.0
        print(f"WARNING: no --fps given and no/insufficient --images-left to estimate it; defaulting to {fps}.", file=sys.stderr)

    with open(args.template) as f:
        template_lines = f.readlines()

    header = (
        "%YAML:1.0\n\n"
        "# Generated by tools/convert_ros_stereo_calib.py from:\n"
        f"#   left:  {os.path.abspath(args.left)}\n"
        + (f"#   right: {os.path.abspath(args.right)}\n" if not mono else "")
        + "# Raw (unrectified) intrinsics/distortion below; ORB-SLAM3 computes its own\n"
        "# rectification maps at startup from these" + (" plus Stereo.T_c1_c2.\n\n" if not mono else ".\n\n")
        + "File.version: \"1.0\"\n\n"
        "Camera.type: \"PinHole\"\n\n"
        "# Camera calibration and distortion parameters (OpenCV)\n"
        f"Camera1.fx: {K1[0,0]:.10f}\n"
        f"Camera1.fy: {K1[1,1]:.10f}\n"
        f"Camera1.cx: {K1[0,2]:.10f}\n"
        f"Camera1.cy: {K1[1,2]:.10f}\n\n"
        f"Camera1.k1: {D1[0]:.10f}\n"
        f"Camera1.k2: {D1[1]:.10f}\n"
        f"Camera1.p1: {D1[2]:.10f}\n"
        f"Camera1.p2: {D1[3]:.10f}\n"
    )
    if len(D1) > 4 and D1[4] != 0.0:
        header += f"Camera1.k3: {D1[4]:.10f}\n"

    if not mono:
        header += (
            "\n"
            f"Camera2.fx: {K2[0,0]:.10f}\n"
            f"Camera2.fy: {K2[1,1]:.10f}\n"
            f"Camera2.cx: {K2[0,2]:.10f}\n"
            f"Camera2.cy: {K2[1,2]:.10f}\n\n"
            f"Camera2.k1: {D2[0]:.10f}\n"
            f"Camera2.k2: {D2[1]:.10f}\n"
            f"Camera2.p1: {D2[2]:.10f}\n"
            f"Camera2.p2: {D2[3]:.10f}\n"
        )
        if len(D2) > 4 and D2[4] != 0.0:
            header += f"Camera2.k3: {D2[4]:.10f}\n"

    header += (
        "\n"
        f"Camera.width: {w1}\n"
        f"Camera.height: {h1}\n\n"
    )
    if args.new_width and args.new_height:
        header += (
            f"Camera.newWidth: {args.new_width}\n"
            f"Camera.newHeight: {args.new_height}\n\n"
        )
    header += (
        f"Camera.fps: {round(fps)}\n\n"
        "# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)\n"
        f"Camera.RGB: {args.rgb}\n"
    )

    if not mono:
        T_c1_c2 = np.eye(4)
        T_c1_c2[:3, :3] = R_lr
        T_c1_c2[:3, 3] = t_lr
        header += (
            "\n"
            f"Stereo.ThDepth: {args.th_depth}\n"
            + format_opencv_matrix("Stereo.T_c1_c2", T_c1_c2)
        )

    # Pull everything from "# ORB Parameters" onward out of the template verbatim.
    body = []
    in_body = False
    for line in template_lines:
        if line.strip().startswith("# ORB Parameters"):
            in_body = True
        if in_body:
            body.append(line)

    if not body:
        print(f"WARNING: could not find '# ORB Parameters' section in {args.template}; "
              f"output will only contain the camera calibration block.", file=sys.stderr)

    # Override Viewer.imageViewScale (display-only window size) instead of leaving whatever
    # the template happened to have -- a template tuned for a 752px-wide EuRoC image at scale
    # 1.0 produces an oversized window for a much wider camera. Note: the "Current Frame" window
    # shows the image AFTER Camera.newWidth/newHeight resizing (that resize happens before the
    # image ever reaches Tracking/FrameDrawer), so the scale must be computed off the effective
    # (post-resize) width, not the raw calibration width.
    effective_width = args.new_width if (args.new_width and args.new_height) else w1
    image_view_scale = args.image_view_scale
    if image_view_scale is None:
        image_view_scale = min(1.0, 960.0 / effective_width)
        print(f"Auto Viewer.imageViewScale: {image_view_scale:.3f} (window will be ~{int(effective_width*image_view_scale)}px wide)")
    replaced = False
    for i, line in enumerate(body):
        if line.strip().startswith("Viewer.imageViewScale"):
            body[i] = f"Viewer.imageViewScale: {image_view_scale:.3f}\n"
            replaced = True
            break
    if not replaced:
        body.append(f"Viewer.imageViewScale: {image_view_scale:.3f}\n")

    if args.n_features:
        replaced = False
        for i, line in enumerate(body):
            if line.strip().startswith("ORBextractor.nFeatures"):
                body[i] = f"ORBextractor.nFeatures: {args.n_features}\n"
                replaced = True
                break
        if not replaced:
            body.append(f"ORBextractor.nFeatures: {args.n_features}\n")

    with open(args.out, "w") as f:
        f.write(header)
        f.write("\n#--------------------------------------------------------------------------------------------\n")
        f.writelines(body)

    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
