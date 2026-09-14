# EasyOBJD

FTC object detection, simplified.

[![Release](https://img.shields.io/github/v/tag/IamAki123/EasyOBJD?label=release)](https://github.com/IamAki123/EasyOBJD/tags)
[![JitPack](https://jitpack.io/v/IamAki123/EasyOBJD.svg)](https://jitpack.io/#IamAki123/EasyOBJD)
[![Tests](https://github.com/IamAki123/EasyOBJD/actions/workflows/tests.yml/badge.svg)](https://github.com/IamAki123/EasyOBJD/actions/workflows/tests.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## About EasyOBJD

EasyOBJD turns an EasyOpenCV webcam frame into filtered, camera-relative game-piece positions (X, Y in inches). You configure HSV, ball size, and the camera **once** in a TeamCode file, tune from the Driver Station, then reuse that setup in TeleOp and auto.

It does **not** replace odometry. Camera X/Y do not need a pose. Field X/Y update only after you call `setRobotPose`.

### Why EasyOBJD instead of a raw HSV OpMode?

EasyOpenCV already gives you a `Mat`. EasyOBJD adds the pieces teams usually rewrite:

- HSV mask with live widen/tighten (not hardcoded to one season)
- 12×12 grid clustering so touching pieces are one group
- Circularity + arc-split so peanut blobs become individual balls
- Floor-plane inches from camera height + tilt, with size-based fallback
- Immutable snapshots, intake pick, debug overlay

It does **not** replace a Limelight or a custom ML pipeline if you already trust those. Use EasyOBJD when you want “where is that yellow (or other-color) circle?” in inches on a Control Hub webcam.

**Expected accuracy:** measure *your* robot. With a taped lens height/tilt, a real ball diameter, and a clean mask at 2–4 ft, **a couple of inches** of Y error is a common good result — not a guarantee. Lighting, glare, and wide-angle distortion dominate. Error that *grows toward the image edge* is usually an undistorted lens, not a missing filter. How to score tape vs vision: [Tuning](docs/Tuning.md).

**Current release:** 1.0.0. Copy-in `EasyOBJDUserConfig`, Tuner, Calibrate. JitPack: `com.github.IamAki123:EasyOBJD:1.0.0`. Formerly **EasyOBD** (`EasyOBD*` types still compile).

## First time here?

Do these in order. Each step has a longer page if you get stuck.

| Step | What you do | Details |
| --- | --- | --- |
| 1 | Add the JitPack dependency and EasyOpenCV, sync Gradle | [Install](docs/Install.md) |
| 2 | Copy [`EasyOBJDUserConfig`](samples/EasyOBJDUserConfig.java) into TeamCode. Set webcam name, HSV, ball size, camera height/tilt | Copying the file does nothing by itself — OpModes must call `EasyOBJDUserConfig.create()` |
| 3 | Measure the **lens** height and tilt. Set `BALL_DIAMETER_INCHES` to the official piece | Filters cannot fix wrong geometry |
| 4 | Run **EasyOBJD Tuner** (D-pad up = wider, down = tighter). Copy HSV back into UserConfig | [Tuning](docs/Tuning.md) |
| 5 | Run **EasyOBJD Calibrate** with one ball at a known tape distance. Copy focal / tilt | [Tuning](docs/Tuning.md#2-inches--easyobjd-calibrate) |
| 6 | Run **EasyOBJD Sample**. When tape and Y agree, use `getBestClusterForIntake()` in TeleOp | [Sample OpMode](docs/SampleOpMode.md) |

```
EasyOBJDUserConfig (TeamCode)
        HSV, ball size, camera, webcam
                │
                ▼
        Tuner (practice)  →  Calibrate (practice)  →  match TeleOp / auto
                             getClusters() / getBestClusterForIntake()
```

The tuners are **not** in the JitPack AAR.

## 1. Install

In a stock FTC SDK project, add JitPack next to `mavenCentral()` and `google()` in the **root** `build.dependencies.gradle`:

```gradle
repositories {
    mavenCentral()
    google()
    maven { url = 'https://jitpack.io' }
}
```

Then in `TeamCode/build.gradle`, inside `dependencies`:

```gradle
implementation 'org.openftc:easyopencv:1.7.3'
implementation 'com.github.IamAki123:EasyOBJD:1.0.0'
```

**Sync:** File → Sync Project with Gradle Files. Use Android Studio’s Embedded JDK for the Gradle JVM.

Sync errors: [Install](docs/Install.md).

## 2. Configure the robot once

> ‼️ **Point TeleOp and auto at `EasyOBJDUserConfig`.** Copying the file does nothing if those OpModes still call `EasyOBJD.createPipeline()` with no config — you keep AAR yellow defaults (`Webcam 1`, 2.8 in, 19 in / 25°).

Copy [`samples/EasyOBJDUserConfig.java`](samples/EasyOBJDUserConfig.java) into TeamCode. Edit **that** file. You do not edit the library.

```java
public static String WEBCAM_NAME = "Webcam 1";   // must match Configure Robot
public static double BALL_DIAMETER_INCHES = 2.8;

public static double H_LOW = 21, S_LOW = 95, V_LOW = 85;
public static double H_HIGH = 35, S_HIGH = 255, V_HIGH = 255;

public static double CAMERA_HEIGHT_INCHES = 19.0;
public static double CAMERA_TILT_DEGREES = 25.0;  // positive = pitched down
```

Then:

```java
EasyOBJDPipeline pipeline = EasyOBJD.createPipeline(EasyOBJDUserConfig.create());
webcam.setPipeline(pipeline);
```

### Coordinates (read once)

| Quantity | Units / convention |
| --- | --- |
| Cluster X / Y | Inches right / forward of the **lens** |
| Robot X / Y | Inches right / forward of robot center (offsets + yaw) |
| Field X / Y | After `setRobotPose` (inches, heading radians) |
| HSV hue | OpenCV 8-bit **0–179** |
| Camera tilt | Degrees, **positive = down** |
| Ball diameter | Inches |

## 3. Use it in an OpMode

Do not paste HSV or ball size into every OpMode. Call `EasyOBJDUserConfig.create()`.

```java
pipeline = EasyOBJD.createPipeline(EasyOBJDUserConfig.create());

// every loop, optional:
// pipeline.setRobotPose(pose.getX(), pose.getY(), pose.getHeading());

ClusterInfo intake = pipeline.getBestClusterForIntake();
if (intake != null) {
    // intake.x / intake.y — inches from the lens
}
```

Drive and telemetry: [Sample OpMode](docs/SampleOpMode.md). Builder details: [API](docs/API.md).

## 4. Tune (practice, not matches)

1. Driver Station → **EasyOBJD Tuner**. D-pad **up** = wider HSV, **down** = tighter. Copy the printed `H_LOW`…`V_HIGH` into `EasyOBJDUserConfig`.
2. **EasyOBJD Calibrate**: one ball, known tape distance, on-axis. Copy focal length and tilt.
3. Change one value at a time.

Tape the **lens** first. [Tuning](docs/Tuning.md).

## Docs

| Page | When to open it |
| --- | --- |
| [Docs index](docs/DocsInfo.md) | List of all guide pages |
| [Prerequisites](docs/Prerequisites.md) | Webcam, what EasyOBJD does not do |
| [Install](docs/Install.md) | Gradle, JitPack, local module, JDK |
| [Sample OpMode](docs/SampleOpMode.md) | Files to copy |
| [Tuning](docs/Tuning.md) | Tuner vs Calibrate, field procedure |
| [Math (simple)](docs/MathButDumbed.md) | How it works, no formulas |
| [Math](docs/Math.md) | Pinhole, tilt, floor-plane |
| [What each file does](docs/LibraryFiles.md) | Pipeline, config, math, samples |
| [API](docs/API.md) | Method-by-method reference |
| [Troubleshooting](docs/Troubleshooting.md) | Blank mask, wrong inches, NaN field |
| [Changelog](CHANGELOG.md) | What changed between releases |
| [Contributing](CONTRIBUTING.md) | Building this repo from source |

## Credits

Akash Vijay Aradhya — #23918 Super Sigma Robotics

AI tools (Cursor, ChatGPT, OpenAI Codex in Cursor) were used as development assistants for code generation, debugging, documentation, and refinement. Architecture, requirements, testing, validation, and final implementation decisions were directed and reviewed by the author.
