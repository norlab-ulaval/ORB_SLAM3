# Modifications vs. upstream ORB-SLAM3

This documents everything this fork (branch `bracketing`, diverged from `master` at
`7816c4a`) changed relative to upstream ORB-SLAM3, and why. It covers both the five
commits already on the branch and the still-uncommitted working-tree changes made while
diagnosing/fixing the exposure-bracketing tunnel-exit tracking break.

Three goals drove almost everything below:
1. Run ORB-SLAM3 stereo on a custom dataset (directory of timestamp-named PNGs + a
   per-frame exposure-bracketing metadata CSV), captured specifically to survive
   extreme-dynamic-range transitions (e.g. exiting a tunnel) by cycling exposure
   (dark / mid / bright / mid) every 4 frames.
2. Diagnose and fix a reproducible trajectory reset that happened exactly at a
   tunnel-exit transient, without resorting to deep-learning stereo (explicitly out of
   scope) or discarding any of the bracket-cycle exposures (also explicitly out of
   scope — the dark exposure is precisely what should survive the transient).
3. Once resets were eliminated, diagnose and fix a subtler problem on a second dataset:
   bracketing tracked with zero resets but was measurably *less smooth* than a plain
   constant-exposure run over the identical physical route — the opposite of what more
   exposure information should buy (§6).

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

## 6. Per-phase descriptor coverage & keyframe match-yield gate (July 31 stair dataset)

A second, independent dataset pair (`yoda_july31_stair_bracketing`/`yoda_july31_stair_ae`,
captured back-to-back at the same physical location, both with LiDAR ground truth) surfaced
a different problem than the tunnel-exit reset: the bracketing run tracked with zero resets
but was visibly **less smooth** than a classical constant-exposure AE run over the identical
route — the opposite of what more exposure diversity should buy. A same-physical-loop
trajectory overlay showed AE returning close to its starting height by the end of a ~44m
loop, while bracketing drifted to a spurious ≈−2.8m by the end.

**Investigation (see `tools/bracketing_diag/analyze_phase_quality.py`, new)** ruled out
several plausible-sounding hypotheses before finding the real one — each tested directly
against instrumented data, not assumed:
- **Not clustered/sparse keypoints.** A per-frame spatial-spread diagnostic (grid-occupancy
  of valid-depth keypoints, added to `ORBSLAM_DIAG_STEREO`'s output) showed 94–99% grid
  coverage and hundreds of valid-depth points for *every* phase, including the extremes.
- **Not blur or noise in the expected direction.** A sharpness diagnostic
  (`ORBSLAM_DIAG_SHARPNESS`, new, in `stereo_general.cc`) showed dark frames with the
  *lowest* Laplacian variance and bright frames the *highest* — the reverse of the
  naive "long exposure → blur, short exposure → noise" prediction.
- **Not map corruption from a bad keyframe.** `ORBSLAM_DIAG_KF` (new) showed keyframes
  created during the bad stretch had normal-to-good point counts and spread; post-stretch
  quality metrics didn't degrade.
- **Not the same-phase fallback.** `trackMethod` stayed `motion_model` throughout the worst
  window — ordinary tracking, not a fallback/relocalization path, was producing the drift.
- A visible discrete "jump" in the trajectory overlay traced to one frame (dark-phase,
  `id=633` in the diagnostic run) whose *own* extracted keypoints looked completely normal
  by every above metric, yet whose `TrackLocalMap` match count and post-optimization inlier
  count were roughly half its neighbors' (189 matches/128 inliers vs. neighbors' 270–400/
  280–450) — pointing at the local-map *matching* step itself, not the frame's own image.

**Root cause**: `MapPoint::GetDescriptor(phase)` (§5) falls back to the general,
all-observations descriptor whenever a point has no observation from the query phase.
Under the dark→mid→bright→mid cycle, the mid phase occupies 2 of 4 nominal slots, so dark
and bright are structurally "minority" phases — any given point gets far fewer chances to
ever be observed *during* a dark or bright frame, so `mDescriptorByPhase[0]`/`[2]` stays
empty far longer than `[1]` does. A new diagnostic, `ORBSLAM_DIAG_PHASEDESC` (prints
`PHASE_DESC id=... phase=... nmatches=... withPhaseDesc=... withoutPhaseDesc=...` from
`ORBmatcher::SearchByProjection`, plus `MapPoint::HasPhaseDescriptor(phase)` to detect the
fallback without changing `GetDescriptor`'s behavior), confirmed this run-wide, not just at
the one visible spike: dark/bright frames fall back to the compromise descriptor 44–49% of
the time vs. mid's 32%, and `fallbackRate` correlates with `nmatches` at **r=−0.52** and
with `mnMatchesInliers` at **r=−0.50** across the whole run (1227 frames). Critically, the
*inlier rate* (inliers ÷ total matches) is flat across phases at ~89% — matches that are
found are just as likely to be correct. The problem is **yield**, not correctness: dark and
bright frames systematically start pose optimization with fewer, less-constraining
correspondences, producing a small precision penalty on every one of their frames that
compounds into visible drift over hundreds of frames, without any single frame ever looking
obviously bad by count alone. Classical AE never pays this cost since it has only one phase,
so every observation of every point counts toward its one descriptor.

Two fixes, in `src/MapPoint.cc` and `src/Tracking.cc`:

1. **Widened descriptor fallback** (`MapPoint::GetDescriptor(int phase)`): before falling
   all the way to the general compromise descriptor, try the *other* minority phase's own
   descriptor (dark↔bright) if it has one. BRIEF/ORB descriptors compare relative (ordinal)
   pixel-pair intensities within a patch specifically to be robust to global brightness
   shifts, so a genuinely dark-phase-specific descriptor should still resemble a
   bright-phase query better than a mid-dominated average would.
2. **Keyframe match-yield gate** (`Tracking::CreateNewKeyFrame()`): a single, non-phase-keyed
   trailing history of the last 20 frames' `mnMatchesInliers` (`Tracking::mRecentMatchesInliers`,
   updated unconditionally in `TrackLocalMap()`). If a keyframe's own inlier count is below
   `ORBSLAM_KF_YIELD_MIN_RATIO` (default 0.4) of the trailing median, skip triangulating
   *new* MapPoints from it — existing points, the `KeyFrame` object itself,
   `mpReferenceKF(ByPhase)`, and all per-phase tracking state are untouched, so a
   low-yield frame still contributes to tracking continuity, it just doesn't get to permanently
   seed new, less-precisely-triangulated points into the map.

Both fixes were explicitly checked against the reverted velocity-guard's failure mode (a
per-phase state cascade — see §5's "Net measured effect" history): neither reads or writes
`mVelocity(ByPhase)` or anything the same-phase fallback chain consumes, and the one new
piece of state (`mRecentMatchesInliers`) is a single instance shared across all phases, so
there's no per-phase partition of it to selectively starve.

**Measured effect** (`stair_bracketing`, RPE vs. LiDAR ground truth,
`tools/bracketing_diag/compute_rpe.py`):

| | before | after |
|---|---|---|
| Translational RPE RMSE | 0.256 m | 0.235 m |
| Translational RPE max | **1.75 m** | **0.62 m** |
| Rotational RPE RMSE | 3.04° | 3.11° |
| Rotational RPE max | 13.4° | 12.4° |
| Final spurious height drift | ≈−2.8 m | ≈−0.24 m |
| Resets | 0 | 0 |

Neutrality confirmed on `stair_ae` (single-phase dataset — fix 1 is a structural no-op
there, fix 2 exercised but harmless): RPE unchanged within noise (0.227m→0.227m RMSE). No
regression on the July 18 tunnel dataset: still zero resets, one unified map, fail count
(64) within the previously-validated 37–169 range.

## 7. Diagnostic instrumentation (env-var gated, zero cost when unset)

All read once via `getenv(...)` into a `static const` — no measurable overhead when the
variable isn't set, no signature changes, and safe to leave permanently in the tree.

| Env var | Where | Prints |
|---|---|---|
| `ORBSLAM_LOG_FRAMES=1` | `stereo_general.cc` | `FRAME ni=<index> ts_ns=<timestamp>` per frame |
| `ORBSLAM_DUMP_FRAMES_DIR=<dir>` (+ `_START_NS`/`_END_NS`) | `stereo_general.cc` | writes annotated frame PNGs for offline video export |
| `ORBSLAM_DIAG_STEREO=1` | `Frame::ComputeStereoMatches` | `STEREO_MATCH phase=... N=... validDepth=... candidateDisparities=... gridOccupied=... gridTotal=... bboxFracX=... bboxFracY=... ts=...` per frame. Note `phase` always prints `-1` here — `mnExposurePhase` is set *after* the `Frame` constructor returns, so correlating this line to a phase requires an external join on `ts` against the metadata CSV. `gridOccupied`/`bboxFrac*` (§6) measure spatial spread of valid-depth keypoints. |
| `ORBSLAM_DIAG_SAMEPHASE=1` | `TrackReferenceKeyFrameSamePhase`, `TrackWithMotionModelSamePhase` | `SAMEPHASE_RKF`/`SAMEPHASE_MM`/`SAMEPHASE_MM_RESULT` with `nmatches`/`nmatchesMap`/`refAgeSec`/`refMPs` |
| `ORBSLAM_DIAG_TRACKMETHOD=1` | `Tracking::Track()` | `TRACK_METHOD id=... method=... state=... phase=... transJump=... rotJumpDeg=... ts=...` per accepted frame — which function produced the pose, and the raw frame-to-frame pose delta. |
| `ORBSLAM_DIAG_SHARPNESS=1` | `stereo_general.cc` | `SHARPNESS ni=... phase=... lapVarLeft=... lapVarRight=... meanIntensityLeft=... ts_ns=...` — blur (Laplacian variance) and brightness of the raw captured image, before any rectification. |
| `ORBSLAM_DIAG_KF=1` | `Tracking::CreateNewKeyFrame()` | `NEW_KF id=... phase=... nNewPoints=... nReusedPoints=... ts=...` |
| `ORBSLAM_DIAG_PHASEDESC=1` | `ORBmatcher::SearchByProjection(Frame&, ...)`, `Tracking::TrackLocalMap()` | `PHASE_DESC id=... phase=... nmatches=... withPhaseDesc=... withoutPhaseDesc=... ts=...` and `TLM_INLIERS id=... phase=... nMatchedTotal=... mnMatchesInliers=... ts=...` — see §6. |
| `ORBSLAM_KF_YIELD_MIN_RATIO=<float>` | `Tracking::CreateNewKeyFrame()` | Not a diagnostic — tunes §6's keyframe match-yield gate threshold (default 0.4) without a rebuild. |
| `ORBSLAM_KF_BURST=1` | `Tracking::NeedNewKeyFrame()`/`CreateNewKeyFrame()` | **Experimental, off by default, not shipped** — see §9. |

## 8. Module-level summary (quick reference)

| Module | New/Modified | One-line summary |
|---|---|---|
| `Examples/Stereo/stereo_general.cc` | New | Directory-based stereo dataset runner; exposure-phase loader; headless viewer toggle; frame logging/dumping diagnostics |
| `Examples/Monocular/mono_general.cc` | New | Monocular counterpart, no phase logic |
| `Examples/Stereo/yoda*.yaml`, `Examples/Monocular/yoda.yaml` | New | This rig's calibration/settings files |
| `tools/convert_ros_stereo_calib.py` | New | ROS calibration YAML → ORB-SLAM3 settings YAML converter |
| `tools/bracketing_diag/` | New (untracked) | Ad-hoc analysis scripts, run logs, and videos accumulated while diagnosing the tunnel-exit break (not part of the shipped fix) |
| `include/Frame.h`, `src/Frame.cc` | Modified | `mnExposurePhase` tag; stereo sub-pixel confidence gate (root-cause fix, §4); `ORBSLAM_DIAG_STEREO` |
| `include/KeyFrame.h`, `src/KeyFrame.cc` | Modified | `mnExposurePhase` tag, propagated from constituent `Frame` |
| `include/MapPoint.h`, `src/MapPoint.cc` | Modified | Per-phase representative descriptor (`mDescriptorByPhase`) + `GetDescriptor(phase)`; widened dark↔bright fallback + `HasPhaseDescriptor(phase)` (§6) |
| `src/ORBmatcher.cc` | Modified | `SearchByProjection` (local-map variant) uses phase-aware descriptor lookup; `ORBSLAM_DIAG_PHASEDESC` (§6) |
| `include/System.h`, `src/System.cc` | Modified | `TrackStereo(..., exposurePhase)`; new `GetFrameDrawerImage()` accessor for headless viewer capture |
| `include/Tracking.h`, `src/Tracking.cc` | Modified | Phase-tag threading; per-phase last-frame/reference-KF/velocity state; `TrackReferenceKeyFrameSamePhase`/`TrackWithMotionModelSamePhase`/`TrySamePhaseFallback`; wired into `Track()`'s steady-state fallback and `RECENTLY_LOST` recovery; `time_recently_lost` unification; keyframe match-yield gate + `mRecentMatchesInliers` (§6); `ORBSLAM_KF_BURST` experiment (§9, not shipped) |
| `src/FrameDrawer.cc` | Modified | Stereo-depth-outcome debug overlay (yellow/red); `RECENTLY_LOST` render branch (§3) |
| `src/Settings.cc` | Modified | Fixed wrong image size passed to `cv::stereoRectify` when downsampling (unrelated bug fix) |
| `CMakeLists.txt` | Modified | C++14, OpenCV 4.2, new executable targets |
| `docker/`, `.devcontainer/` | New/modified | Dev environment: prebuilt-Pangolin image, fast entrypoint, VS Code devcontainer |
| `tools/bracketing_diag/compute_rpe.py`, `analyze_phase_quality.py` | New | RPE-vs-ground-truth evaluation; per-phase match-quality analysis (§6) |

## 9. Explicitly considered and NOT done

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
- **Velocity-guard on same-phase-fallback frames** (§6 investigation) — suppressing
  `mVelocity(ByPhase)` updates after a stale-reference recovery, intended to stop that
  frame's position error from being extrapolated forward. Reverted: because a hard
  stretch cycles through all phases every frame, this starved `mbVelocityByPhase` for
  *all four phases* within a few frames, collapsing the whole fallback chain down to
  raw `Relocalization()` (no temporal continuity), which oscillated for 37 consecutive
  frames between two ~6m-apart candidate poses. The postmortem comment lives directly
  in `Tracking.cc`'s velocity-update block and shaped how §6's two fixes were
  deliberately designed (single, non-phase-keyed state only) to avoid the same cascade.
- **Unconditional keyframe burst** (`ORBSLAM_KF_BURST`, still in the tree, off by
  default) — forcing every keyframe decision to also promote the rest of the current
  bracket cycle to keyframes. A refined, recovery-triggered-only variant measurably
  improved both early-segment straightness and tunnel-exit smoothness in a truncated-
  dataset test, but was superseded by §6's more targeted, better-evidenced fix before
  going through full-dataset validation — left as an unshipped experiment rather than
  deleted, in case future data suggests reviving it.
- **Replacing ORB with a learned feature detector/descriptor** (e.g. SuperPoint/DISK) —
  considered when discussing §6's findings. Not pursued: §6 showed the *inlier rate*
  or matched keypoints is unaffected by exposure phase (~89% flat across phases) — the
  problem is per-phase descriptor *coverage*, not descriptor *quality* — so a better
  descriptor wouldn't target the actual mechanism. Would also require CUDA/inference
  runtime infrastructure this project doesn't have (same gap as the deep-learning
  stereo option above) and touch the BoW vocabulary and Hamming-distance matching
  throughout the codebase, a much larger change than anything else in this document.
