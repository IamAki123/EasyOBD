# EasyOBD

FTC EasyOpenCV pipeline that finds yellow game-piece **clusters**, splits merged blobs into circular balls, and reports each cluster’s camera-relative **X / Y** in inches (optional field frame after you push a robot pose).

JitPack: `com.github.IamAki123:EasyOBD:0.2`

## Install

In `build.dependencies.gradle` (or your root repositories):

```gradle
maven { url = 'https://jitpack.io' }
```

In `TeamCode/build.gradle`:

```gradle
implementation 'com.github.IamAki123:EasyOBD:0.2'
implementation 'org.openftc:easyopencv:1.7.3'
```

Or, while developing next to this repo:

```gradle
implementation project(':EasyOBD')
```

**Compatibility:** EasyOpenCV **1.7.x**, FTC RobotCore / SDK **10.x or 11.x** (built against RobotCore 11.1.0). Java 8. The library is `compileOnly` on those artifacts so your TeamCode SDK version wins at runtime.

## Use

Copy [`samples/EasyOBDSample.java`](samples/EasyOBDSample.java) into TeamCode, or:

```java
EasyOBDConfig config = EasyOBDConfig.builder()
        .camera(19.0, 25.0, 70.4)   // height in, downward tilt deg, horizontal FOV deg
        .processWidth(320)          // process at 320-wide; preview can stay 640×480
        .build();

EasyOBDPipeline pipeline = EasyOBD.createPipeline(config);
webcam.setPipeline(pipeline);

// Camera frame (no odometry):
for (ClusterInfo cluster : pipeline.getClusters()) {
    // cluster.id is 1..n, left to right
    // cluster.x = inches right of the lens
    // cluster.y = inches forward of the lens
    // cluster.localization = FLOOR_PLANE or SIZE_BASED
    // cluster.balls = circular pieces associated with this group
}

// Optional field frame (Pedro / pinpoints / OTOS):
pipeline.setRobotPose(localizer.getX(), localizer.getY(), localizer.getHeading());
double fieldX = pipeline.FieldX.bestClusterCenterpoint();
double fieldY = pipeline.FieldY.bestClusterCenterpoint();

ClusterInfo intake = pipeline.getBestClusterForIntake(); // closest by default
```

Live HSV: **D-pad up** widens yellow, **D-pad down** tightens (`pipeline.adjustHsvRange`). **X** cycles the debug overlay. Tuned starting range:

- Lower: `21, 95, 85`
- Upper: `35, 255, 255`

`getClusters()` / `getBalls()` return immutable snapshots from the last frame. Do not mutate them.

## Coordinate frames

```
Camera (no pose required)
          +Y forward of lens
              ^
              |
              |
    −X <------+------> +X right of lens
            lens

Robot (offsets + yaw from EasyOBDConfig)
          +Y forward of robot center
              ^
              |
    −X <------+------> +X right of center

Field (after setRobotPose, Pedro-style)
    +X forward on the field when heading = 0
    +Y left-handed partner axis used by cameraToField
```

1. **Camera X / Y** — pinhole + tilt. X is right of the lens, Y is forward along the floor. This is what `ClusterInfo.x` / `.y` and `getClusterX()` / `getClusterY()` report.
2. **Robot X / Y** — camera point rotated by `cameraYawDegrees` and shifted by `cameraForwardOfCenter` / `cameraRightOfCenter`. See `getClusterRobotX()` / `getClusterRobotY()`.
3. **Field X / Y** — robot point rotated by the heading you pass to `setRobotPose`. `ClusterInfo.fieldX` / `.fieldY` are `NaN` until a pose is set.

**Localization method:** floor-plane (camera height + downward tilt) is used when the pixel ray hits the ball-center plane inside `maxRangeInches`. Size-based ranging (known 2.8 in diameter vs apparent radius) is the fallback. Read `cluster.localization` or `pipeline.usedFloorPlane()`.

## Calibration

Copy [`samples/EasyOBDCalibrateSample.java`](samples/EasyOBDCalibrateSample.java) or use `EasyOBDCalibration` directly.

1. Measure **camera height** from the tiles to the lens (tape).
2. Place **one official ball** on the floor, centered in the image, at a known tape distance *D* (31 in is a good start).
3. Read the apparent radius *r* (px) from the calibrate sample / debug overlay.
4. Set:

```java
config.focalLengthPixelsAt640 = EasyOBDCalibration.focalLengthAt640(r, D, processWidth);
config.cameraTiltDegrees = EasyOBDCalibration.suggestedTiltDegrees(heightIn, D);
```

`suggestedTiltDegrees` assumes the ball sits on the optical axis (image center). If it does not, put a phone inclinometer on the camera housing and use that pitch instead.

Leave `focalLengthPixelsAt640 = 0` to derive focal length from `horizontalFovDegrees` (default 70.4° → ~454 px at 640-wide).

Checkerboard / AprilTag extrinsics can replace these tape numbers later; the math hook is `setRobotPose` plus `EasyOBDConfig` camera offsets.

## HSV, tilt, and lighting

- **Widen / tighten** with D-pad, or set `config.yellowLower` / `yellowUpper`.
- Enable `config.adaptiveLighting` if arena lights punch holes in the mask (relaxes S/V when the frame is dark).
- Additional colors: `config.extraColorRanges.add(new EasyOBDConfig.ColorRange(lower, upper))`.
- Morphology kernel size scales with process width (override with `openKernelSize` / `closeKernelSize`).

## Resolution and performance

| Stream (preview) | `processWidth` | Typical Control Hub cost | When to use |
| --- | --- | --- | --- |
| 640×480 | 640 (or scale 1.0) | ~20–40 ms / frame | Distant balls, first bring-up |
| 640×480 | **320** | ~8–20 ms / frame | Driving / intake (recommended) |

Preview stays at stream resolution. Processing is independent when `processWidth` is set. Times vary with lighting and blob count; read `pipeline.getLastProcessTimeMs()` or enable the sample `DEBUG_MODE`.

## What it does

1. HSV yellow mask (optional extra ranges, adaptive lighting)
2. Morphological open/close (size scales with resolution) + fill external holes
3. 12×12 integral-image grid; touching occupied cells become a cluster (grid shrinks on tiny frames)
4. Circular / arc ball detection (Kasa fit + voting). High-circularity contours skip the expensive split. Weak arcs are dropped before merge.
5. Floor-plane X/Y from camera height + pitch, or size-based fallback
6. Optional exponential smoothing (`smoothingAlpha`) for driving; leave `0` for raw high-speed data

## Known limitations

- Harsh arena lighting, glare, and reflective tiles still create holes or false yellow.
- Partial occlusion and very distant balls (> `maxRangeInches`, default 60 in) are rejected.
- Wide-angle / fisheye lenses are not undistorted; detections near the image edge will bias range.
- Two balls that completely merge into one circle may still report as one.
- Floor-plane needs a downward tilt. A level camera at 19 in cannot see a floor ball at 31 in on the optical axis.

## API notes

- **Config is per pipeline.** Statics such as `EasyOBDPipeline.CAMERA_HEIGHT_INCHES` are defaults copied by the no-arg constructor. After construction, change `pipeline.getConfig()` (or pass an `EasyOBDConfig`).
- **`EasyOBDCreation`** is a deprecated subclass of `EasyOBDPipeline` for 0.1 source compatibility.
- **`getBestClusterForIntake(CLOSEST | LEFTMOST | HIGHEST_CONFIDENCE)`** applies a simple team heuristic.
- **Overlay:** `OverlayMode.FULL`, `MASK`, `GRID`, `BALLS`, `DISTANCES`.
- Offline: you can pass a pre-built `Mat` into `processFrame`. The JUnit tests in `src/test/java` cover pinhole + tilt math and grid clustering without loading OpenCV.

## Samples

| File | Purpose |
| --- | --- |
| [`samples/EasyOBDSample.java`](samples/EasyOBDSample.java) | TeleOp: camera + field usage, debug overlay, intake pick |
| [`samples/EasyOBDCalibrateSample.java`](samples/EasyOBDCalibrateSample.java) | Tape calibration for focal length and tilt |
