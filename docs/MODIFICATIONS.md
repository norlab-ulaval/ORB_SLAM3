# Modifications vs. upstream ORB-SLAM3

This documents everything this fork (branch `bracketing`, diverged from `master` at
`7816c4a`) changed relative to upstream ORB-SLAM3, and why. It covers both the five
commits already on the branch and the still-uncommitted working-tree changes made while
diagnosing/fixing the exposure-bracketing tunnel-exit tracking break.

Two goals drove almost everything below:
1. Run ORB-SLAM3 stereo on a custom dataset (directory of timestamp-named PNGs + a
   per-frame exposure-bracketing metadata CSV), captured specifically to survive
   extreme-dynamic-range transitions (e.g. exiting a tunnel) by cycling exposure
   (dark / mid / bright / mid) every 4 frames.
2. Diagnose and fix a reproducible trajectory reset that happened exactly at a
   tunnel-exit transient, without resorting to deep-learning stereo (explicitly out of
   scope) or discarding any of the bracket-cycle exposures (also explicitly out of
   scope — the dark exposure is precisely what should survive the transient).

Full diff: `git diff master` from repo root (or `git diff 7816c4a` — same base).

---

## 1. Build & environment

| File | Change | Why |
|---|---|---|
| `CMakeLists.txt` | C++11 → **C++14**; `find_package(OpenCV 3.2)` → **`OpenCV 4.2`**; registered two new executable targets, `stereo_general` and `mono_general` | Host/container ships OpenCV 4.x, not the 3.2 upstream targeted. C++14 is required by Pangolin v0.6's `sigslot` header. |
| `src/Settings.cc` | `cv::stereoRectify(...)` was passed `newImSize_` (the **downsampled** output size) as the *input* calibration image size; changed to `originalImSize_` | Bug fix, unrelated to bracketing. `stereoRectify`'s image-size argument must match the resolution the input `K`/`D` calibration was computed at, not the resolution you're rectifying *to* — passing the downsampled size there miscomputes the rectification maps whenever `Camera.imageScale != 1`. |
| `docker/Dockerfile` | New. Ubuntu 22.04 image; builds Pangolin **v0.6** from source (pinned — v0.7+ requires EGL, which doesn't work with this host's NVIDIA/X11 setup even though it reports no error) | Original repo cloned/built Pangolin at container *entrypoint* time (see below) — every container start = a full Pangolin rebuild. |
| `docker/entrypoint.sh` | Rewritten: no longer clones/builds Pangolin (now baked into the image); decompresses `Vocabulary/ORBvoc.txt.tar.gz` and builds ORB-SLAM3 only if not already present | Fast container startup; idempotent first-run bootstrap. |
| `docker/docker-compose.yaml` | Added `name:`/`image:` keys, renamed container to `orbslam3-cpp` | Working-tree change, cosmetic/naming only. |
| `docker/config.sh` | Deleted | Hardcoded a specific machine's local paths (`~/Documents/ICRA2024/...`); replaced by the compose file's bind mounts. |
| `.devcontainer/devcontainer.json` | New | VS Code devcontainer wired to `docker/docker-compose.yaml`, for editing/building inside the container from an IDE. |
| `.gitignore` | Added `!docker/Dockerfile` exception | The blanket ignore rules would otherwise have excluded the new Dockerfile. |

## 2. New example runners & dataset tooling

Upstream ORB-SLAM3 ships one runner per benchmark dataset format (EuRoC, TUM,
KITTI...), each with its own hardcoded image-listing convention. None of them match
this rig's actual capture layout, so two new general-purpose runners were added instead
of shoehorning into an existing one.

### `Examples/Stereo/stereo_general.cc` (new, 418 lines) / `Examples/Monocular/mono_general.cc` (new, 233 lines)

Expects:
```
<sequence_folder>/images_left/<timestamp_ns>.png
<sequence_folder>/images_right/<timestamp_ns>.png   (stereo only)
<sequence_folder>/images_meta_left/images_meta_left.csv   (optional, stereo only)
```
No separate timestamps file — the filename stem itself *is* the nanosecond timestamp.
Left/right frames are paired by exact filename match; anything without a same-named
counterpart on the other side is dropped (warned, not fatal).

Beyond the loader, `stereo_general.cc` adds:
- **`Viewer.on` yaml key** (`bool bUseViewer`, read via `cv::FileStorage` before
  constructing `System`): toggles the Pangolin viewer without a rebuild. Defaults to on
  if the key is absent, so existing configs are unaffected.
- **`ORBSLAM_LOG_FRAMES=1`**: prints `FRAME ni=<index> ts_ns=<timestamp>` per frame, so
  other stdout diagnostics can be pinned to a raw frame index/timestamp.
- **`ORBSLAM_DUMP_FRAMES_DIR=<dir>`** (+ optional `ORBSLAM_DUMP_START_NS`/
  `ORBSLAM_DUMP_END_NS` to restrict the range): calls the new
  `System::GetFrameDrawerImage()` and writes each frame's annotated (keypoint-overlay)
  image as `frame_%06d.png` — lets a **headless** run still produce the same
  visualization the live Pangolin viewer would show, for offline video export.
- **`LoadExposurePhases(...)`**: reads `images_meta_left.csv`, and for each frame
  timestamp derives an integer *exposure-bracket phase* (0..3) from that frame's
  **actual `exposure_factor` value** — bucketed by distinct value, ascending, so phase 0
  is always the darkest present. Deliberately **not** based on any nominal
  `sequence_index`/cycle-position column: the camera's AE can override the nominal
  slot's exposure during rapid lighting changes (confirmed ~10 frames dataset-wide where
  the nominal "dark slot" was actually shot at 1.0x or 4.0x), and phase needs to reflect
  what the frame actually looks like, not what slot it was scheduled into. A dataset
  with no metadata CSV (or a non-bracketed one) gets phase `-1` (unknown) for every
  frame, which the rest of the pipeline treats as an explicit "not applicable" — see §5.
- Per-frame phase is passed into `SLAM.TrackStereo(..., vExposurePhaseCam[seq][ni])`.

`mono_general.cc` is the same directory-based loader minus the right camera and phase
logic (monocular was never part of the bracketing work).

### `Examples/Stereo/yoda.yaml`, `yoda_downsampled.yaml`, `yoda_july24_downsampled.yaml` / `Examples/Monocular/yoda.yaml` (new)

Settings files for this specific rig's calibration (raw + downsampled variants — the
downsampled ones exist because full-resolution tracking was too slow for
fast iteration during diagnosis). `yoda_downsampled.yaml`'s current working-tree state:
`ORBextractor.nFeatures: 2000` (was 8000, pre-existing change from before the bracketing
work), `Viewer.on: 0` (headless by default).

### `tools/convert_ros_stereo_calib.py` (new, 307 lines)

Converts ROS `camera_calibration`-style stereo calibration YAML (`camera_matrix` /
`distortion_coefficients` / `rectification_matrix` / `projection_matrix`, i.e. K/D/R/P
— which is what this rig's calibration pipeline produces) into an ORB-SLAM3 settings
file. Nontrivial because ORB-SLAM3 wants **raw** (unrectified) per-camera intrinsics
plus the raw inter-camera extrinsic pose, while ROS calibration files only store the
**rectified** R/P outputs, not the raw pose — the script algebraically reconstructs the
raw extrinsic from `R1`, `R2`, and `P2`'s baseline term (derivation in the file's
docstring) and self-checks by round-tripping the result back through
`cv2.stereoRectify` and confirming it reproduces the input R1/R2/P1/P2.

## 3. Debug visualization (`src/FrameDrawer.cc`)

Two independent additions, both confined to `FrameDrawer::DrawFrame()` /
`DrawRightFrame()` / `DrawTextInfo()` — no signature or consumer changes.

**a) Stereo-depth-outcome overlay** (state `OK`): the stock drawer only ever shows
keypoints that became a tracked `MapPoint`/VO point (green/blue). This fork also draws
every *other* extracted keypoint — normally invisible — colored by why it didn't:
- **yellow** = stereo matching found a valid depth for it, but it was never
  matched/associated with a `MapPoint` this frame.
- **red** = stereo matching failed outright (no depth at all).

This was the primary instrument used to diagnose the tunnel-exit break (it's what
directly shows the "hundreds of red dots, few green" pattern discussed with the user).

**b) `RECENTLY_LOST` render branch** (added this session): the stock `DrawFrame()` /
`DrawRightFrame()` only had drawing logic for tracking states `NOT_INITIALIZED` and
`OK` — a third real state, `RECENTLY_LOST`, fell through both the copy-section and the
draw-section `if/else` chains with *no* matching branch, and `DrawTextInfo()` likewise
had no case for it. Net effect: any frame processed while tracking was briefly
`RECENTLY_LOST` rendered as a bare image with **zero** keypoint markers and a
**blank black HUD bar** — regardless of how many keypoints were actually extracted —
which was genuinely misleading when reviewing exported tracking videos (a
well-exposed, feature-rich frame could appear to have "no features" for reasons that
had nothing to do with its image content). Fixed by adding the missing branch
everywhere:
- Copy section: `mState==LOST || mState==RECENTLY_LOST` now both copy `vCurrentKeys`
  (previously `LOST` only).
- Draw section: new `else if(state==Tracking::RECENTLY_LOST)` draws every extracted
  keypoint as a small gray dot (no map-point/depth distinction available in this state).
- `DrawTextInfo()`: new case prints `" RECENTLY LOST - ATTEMPTING RECOVERY "`.

Verified by re-rendering the exact frame that exposed the bug: same raw pixels,
overlay now shows the extracted keypoints (gray) and the correct HUD text instead of a
blank frame.

## 4. Stereo sub-pixel depth confidence gate (`src/Frame.cc::ComputeStereoMatches`)

**This was the actual root-cause fix for the tunnel-exit trajectory reset.**

`ComputeStereoMatches()` computes per-keypoint depth in two stages: (1) a coarse
right-image candidate via ORB descriptor distance, then (2) sub-pixel refinement that
slides an 11×11 patch ±5px and scores each shift by raw, unnormalized SAD
(`cv::norm(IL,IR,NORM_L1)`) on actual pixel intensities, fitting a parabola around the
best-scoring shift. The only pre-existing outlier filter checks *descriptor* distance
from the coarse match — nothing validated the SAD refinement's own quality. Under
low-light/high-sensor-noise conditions, raw intensity SAD can't distinguish real
texture from noise, so it can confidently pick a plausible-but-wrong sub-pixel shift,
which then sails through unfiltered into a permanent, geometrically-wrong 3D point.

Diagnosed via instrumentation (`ORBSLAM_DIAG_STEREO`, see §6) plus match-count logging:
raw 2D frame-to-frame matches against these points routinely reached 40-58 (well above
the 20-match acceptance threshold), but only 0-7 survived pose optimization as
geometric inliers — i.e. the matched points existed but their 3D positions were wrong.

**Fix** (inserted right after the existing `bestincR==-L || bestincR==L` edge check,
before the parabola fit): find the second-best SAD score among candidate shifts,
excluding the best match's immediate neighbors (which the parabola fit already uses and
which naturally score close to the best on any smooth cost curve). Reject the match
(leave `mvDepth`/`mvuRight` at their default -1, i.e. no depth) if the best score isn't
clearly better than that second-best (`bestDist > 0.8 * secondBestDist`) — an ambiguous
match where several candidate shifts look similarly plausible is exactly the noise-vs-
signal confusion described above.

**Result** (4/4 full-dataset validation runs, ~150s/4021 frames): **zero** trajectory
resets anywhere in the dataset, down from an 11-14-reset baseline; tracking-failure
count dropped to 37-169 (avg ~85) from a 747-784 baseline; every run consolidated into
one unified map. This single ~30-line change fully resolved the reported problem —
the exposure-phase-aware tracking machinery in §5 helped independently (see its own
results below) but was not, in the end, necessary for the tunnel-exit reset
specifically; it remains valuable for reducing failure-streak length/severity generally.

Two earlier fix attempts at the same problem were tried and reverted (kept out of the
tree, mentioned here for completeness): a zero-mean-SAD variant (targets left/right
brightness mismatch — this rig's stereo pair is always exposure-synced, confirmed via
CSV cross-check, so there was nothing for it to fix) and a saturation-based image
quality gate in core (`Frame::ComputeImageQualityStats`, never fired because it only
ran after `bOK` was already true, downstream of where the failure actually occurred).

## 5. Exposure-phase-aware tracking

The bracketing capture cycles exposure every 4 frames (dark / mid / bright / mid).
Ordinary ORB-SLAM3 always compares each new frame against whatever frame/keyframe
happened to precede it temporally — during a real illumination transient (tunnel exit),
that immediately-preceding frame might be a wildly different exposure (and therefore a
bad match target) even when a same-exposure frame from a few cycles back would still
match well. This is a multi-file, additive change: a brand new integer tag threaded
through `Frame`/`KeyFrame`/`System`/`Tracking`, new per-phase tracking-reference state,
two new phase-aware tracking functions, a fallback dispatcher, and a phase-aware
`MapPoint` descriptor lookup used during local-map matching. All of it is inert
(`phase == -1` / not STEREO) for any non-bracketed dataset or sensor mode — zero
behavioral change there.

### Phase tag plumbing
- **`include/Frame.h`**: `int mnExposurePhase = -1;` — copied in the copy constructor.
- **`include/KeyFrame.h`**: `int mnExposurePhase;` — set from the constituent `Frame` in
  the main constructor, `-1` in the default constructor.
- **`include/System.h` / `src/System.cc`**: `TrackStereo(...)` gained a trailing
  `int exposurePhase=-1` parameter, threaded straight through to
  `Tracking::GrabImageStereo`.
- **`include/Tracking.h` / `src/Tracking.cc`**: `GrabImageStereo(...)` likewise gained
  `int exposurePhase=-1`; sets `mCurrentFrame.mnExposurePhase = exposurePhase` (STEREO
  branch only).

### Per-phase tracking-reference state (`Tracking.h`/`Tracking.cc`)
New members, all sized `NUM_EXPOSURE_PHASES = 4`:
```cpp
Frame mLastFrameByPhase[4];        bool mbLastFrameByPhaseSet[4];
KeyFrame* mpReferenceKFByPhase[4]; bool mbVelocityByPhase[4];
Sophus::SE3f mVelocityByPhase[4];
```
These are the phase-bucketed counterparts of the ordinary `mLastFrame`/`mpReferenceKF`/
`mVelocity`. Updated in `StereoInitialization()`, `CreateNewKeyFrame()`, and at the end
of `Track()` — all updates gated on `bOK` (the frame's tracking attempt actually
succeeded), never on a failed/predicted pose, to avoid feeding unverified poses into
future same-phase matching. Cleared in `Reset()`/`ResetActiveMap()`.

A subtle bug was found and fixed here mid-session: `mbVelocityByPhase[phase]` was
originally *cleared to false* on every failed frame of that phase. Since nothing else
ever set it back to true except a future *successful* frame of that exact phase, this
silently disabled the phase-aware motion model for an entire failure streak (confirmed
via instrumentation: zero motion-model attempts logged across a 133-frame/5s streak).
Fixed by only updating the velocity *value* on success, never clearing the "have a
velocity" flag on failure — matching how the ordinary (non-phase) `mbVelocity` already
behaves.

### New tracking functions (`Tracking.cc`)
- **`TrackReferenceKeyFrameSamePhase(int phase)`** — mirrors `TrackReferenceKeyFrame()`
  but matches against `mpReferenceKFByPhase[phase]` / `mLastFrameByPhase[phase]` instead
  of the ordinary temporally-adjacent reference.
- **`TrackWithMotionModelSamePhase(int phase)`** — mirrors `TrackWithMotionModel()`
  using `mVelocityByPhase[phase]`/`mLastFrameByPhase[phase]`. Includes a **third,
  wider search pass** (`th = 4×7 = 28`, gated on already having ≥10 matches) beyond the
  stock two-pass widening: a same-phase reference can be several seconds stale during a
  failure streak, and instrumentation showed matches consistently landing at 15-18 —
  just short of the required 20 — against the stock two-pass radius, which is sized for
  a normal ~150ms inter-frame gap, not several accumulated seconds of motion.
- **`TrySamePhaseFallback()`** — tries the current frame's own phase first (motion model,
  then reference-keyframe), then every *other* phase in turn if that fails. Phases are
  numbered by ascending `exposure_factor`, so this naturally tries the darkest remaining
  phase first. Cross-phase fallback matters because a frame's *own* phase can be
  structurally compromised for a whole stretch (e.g. the bright phase staying saturated
  throughout a tunnel-exit transient) even while a different phase is well-exposed for
  the same real-world lighting.

### Wiring into `Track()`
- Steady-state (`mState == OK`): ordinary `TrackWithMotionModel()`/
  `TrackReferenceKeyFrame()` are tried first as before; `TrySamePhaseFallback()` only
  runs if both fail. (An earlier "same-phase-first" ordering was tried and reverted —
  it regressed full-dataset performance, 18 resets vs. an 11-14 baseline.)
- `RECENTLY_LOST` (non-IMU path): `TrySamePhaseFallback()` is now tried **before**
  `Relocalization()`. This is a meaningful gap-fill, not just an ordering tweak:
  `Relocalization()` is a BoW-database lookup that can only succeed against
  *previously-mapped* content, so it's structurally hopeless the first time the camera
  sees genuinely new scenery (e.g. just past a tunnel exit) — the same-phase fallback
  only needs a recent frame/keyframe of *some* phase, which can be from just a few
  frames earlier in the same lost stretch.
- `time_recently_lost`: unified the `RECENTLY_LOST`-state timeout to use this existing
  member (constructor default `5.0`) in both branches that check it, instead of a
  separate hardcoded `3.0f` literal in one of them. (Also tried `6.0` — no measurable
  benefit, reverted to `5.0`.)

### Per-phase `MapPoint` descriptors (`MapPoint.h`/`.cc`, `ORBmatcher.cc`)
`TrackLocalMap`'s `SearchByProjection` step matches the current frame's keypoints
against nearby `MapPoint`s using each point's single "distinctive" descriptor
(`ComputeDistinctiveDescriptors()`'s least-median-Hamming-distance pick across *all*
observations, regardless of exposure). Added:
- `MapPoint::mDescriptorByPhase[4]` — computed alongside the existing `mDescriptor` by
  re-running the same least-median-distance selection, restricted to each phase's own
  subset of observing keyframes. Left empty for phases the point has no observation
  from.
- `MapPoint::GetDescriptor(int phase)` — returns `mDescriptorByPhase[phase]` if
  non-empty, else falls back to the general `mDescriptor` (always safe to call,
  including with `phase == -1`).
- `ORBmatcher::SearchByProjection(Frame&, vector<MapPoint*>&, ...)` (the function
  `TrackLocalMap` calls): both `pMP->GetDescriptor()` call sites changed to
  `pMP->GetDescriptor(F.mnExposurePhase)`.

### Net measured effect
Across the session's iterative validation: the per-phase tracking-reference fixes
(velocity-flag persistence + wider search pass + `RECENTLY_LOST` fallback gap) reduced
full-dataset resets from an 11-14 baseline to ~1-2 on their own, before the §4 stereo
confidence gate was even identified. With both fixes combined, final validation showed
zero resets across the whole dataset (§4's numbers). A controlled experiment (mid-
exposure-only frame stream through the tunnel region) confirmed bracketing is still
*necessary* even with the confidence gate: a mid-only stream still breaks at the tunnel
(11 resets), because the confidence gate can only reject bad depths, not manufacture
scene information a genuinely too-dark/too-bright frame doesn't have.

## 6. Diagnostic instrumentation (env-var gated, zero cost when unset)

All read once via `getenv(...)` into a `static const` — no measurable overhead when the
variable isn't set, no signature changes, and safe to leave permanently in the tree.

| Env var | Where | Prints |
|---|---|---|
| `ORBSLAM_LOG_FRAMES=1` | `stereo_general.cc` | `FRAME ni=<index> ts_ns=<timestamp>` per frame |
| `ORBSLAM_DUMP_FRAMES_DIR=<dir>` (+ `_START_NS`/`_END_NS`) | `stereo_general.cc` | writes annotated frame PNGs for offline video export |
| `ORBSLAM_DIAG_STEREO=1` | `Frame::ComputeStereoMatches` | `STEREO_MATCH phase=... N=... validDepth=... candidateDisparities=... ts=...` per frame. Note `phase` always prints `-1` here — `mnExposurePhase` is set *after* the `Frame` constructor returns, so correlating this line to a phase requires an external join on `ts` against the metadata CSV. |
| `ORBSLAM_DIAG_SAMEPHASE=1` | `TrackReferenceKeyFrameSamePhase`, `TrackWithMotionModelSamePhase` | `SAMEPHASE_RKF`/`SAMEPHASE_MM`/`SAMEPHASE_MM_RESULT` with `nmatches`/`nmatchesMap`/`refAgeSec`/`refMPs` |

## 7. Module-level summary (quick reference)

| Module | New/Modified | One-line summary |
|---|---|---|
| `Examples/Stereo/stereo_general.cc` | New | Directory-based stereo dataset runner; exposure-phase loader; headless viewer toggle; frame logging/dumping diagnostics |
| `Examples/Monocular/mono_general.cc` | New | Monocular counterpart, no phase logic |
| `Examples/Stereo/yoda*.yaml`, `Examples/Monocular/yoda.yaml` | New | This rig's calibration/settings files |
| `tools/convert_ros_stereo_calib.py` | New | ROS calibration YAML → ORB-SLAM3 settings YAML converter |
| `tools/bracketing_diag/` | New (untracked) | Ad-hoc analysis scripts, run logs, and videos accumulated while diagnosing the tunnel-exit break (not part of the shipped fix) |
| `include/Frame.h`, `src/Frame.cc` | Modified | `mnExposurePhase` tag; stereo sub-pixel confidence gate (root-cause fix, §4); `ORBSLAM_DIAG_STEREO` |
| `include/KeyFrame.h`, `src/KeyFrame.cc` | Modified | `mnExposurePhase` tag, propagated from constituent `Frame` |
| `include/MapPoint.h`, `src/MapPoint.cc` | Modified | Per-phase representative descriptor (`mDescriptorByPhase`) + `GetDescriptor(phase)` |
| `src/ORBmatcher.cc` | Modified | `SearchByProjection` (local-map variant) uses phase-aware descriptor lookup |
| `include/System.h`, `src/System.cc` | Modified | `TrackStereo(..., exposurePhase)`; new `GetFrameDrawerImage()` accessor for headless viewer capture |
| `include/Tracking.h`, `src/Tracking.cc` | Modified | Phase-tag threading; per-phase last-frame/reference-KF/velocity state; `TrackReferenceKeyFrameSamePhase`/`TrackWithMotionModelSamePhase`/`TrySamePhaseFallback`; wired into `Track()`'s steady-state fallback and `RECENTLY_LOST` recovery; `time_recently_lost` unification |
| `src/FrameDrawer.cc` | Modified | Stereo-depth-outcome debug overlay (yellow/red); `RECENTLY_LOST` render branch (§3) |
| `src/Settings.cc` | Modified | Fixed wrong image size passed to `cv::stereoRectify` when downsampling (unrelated bug fix) |
| `CMakeLists.txt` | Modified | C++14, OpenCV 4.2, new executable targets |
| `docker/`, `.devcontainer/` | New/modified | Dev environment: prebuilt-Pangolin image, fast entrypoint, VS Code devcontainer |

## 8. Explicitly considered and NOT done

- **Deep-learning stereo matching** (WAFT-Stereo / RAFT-Stereo) — researched and
  architecturally scoped (the codebase's existing `ComputeStereoFromRGBD` dense-depth
  pattern would make integration straightforward), but never implemented per explicit
  direction to stay off deep learning "for now." The cheaper §4 fix resolved the
  problem without it.
- **Dropping/discarding specific bracket exposures** — tried early on, explicitly
  reverted per direction: the whole point of bracketing is to keep every exposure
  available for the transient it's individually good at.
- **Saturation-based core quality gate** (`Frame::ComputeImageQualityStats`) —
  implemented, found to never actually fire (ran downstream of where the real failure
  occurred), reverted per direction.
- **Zero-mean SAD stereo matching** — implemented, found to address a brightness-
  mismatch problem this rig doesn't have (L/R cameras are always exposure-synced),
  reverted.
