# EasyOBJD

FTC object detection, simplified.

[![Release](https://img.shields.io/github/v/tag/IamAki123/EasyOBJD?label=release)](https://github.com/IamAki123/EasyOBJD/tags)
[![JitPack](https://jitpack.io/v/IamAki123/EasyOBJD.svg)](https://jitpack.io/#IamAki123/EasyOBJD)
[![Tests](https://github.com/IamAki123/EasyOBJD/actions/workflows/tests.yml/badge.svg)](https://github.com/IamAki123/EasyOBJD/actions/workflows/tests.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## About EasyOBJD

EasyOBJD turns an EasyOpenCV webcam frame into filtered, camera-relative game-piece positions (X, Y in inches). Copy the sample, D-pad HSV on the Driver Station, read cluster X/Y. Optional UserConfig if you want those numbers saved.

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

**Current release:** 1.0.0. JitPack: `com.github.IamAki123:EasyOBJD:1.0.0`. Copy [`EasyOBJDSample`](samples/EasyOBJDSample.java), run it, D-pad HSV. Optional UserConfig / Calibrate if you want saved inches.

## First time here?

Do these in order. Each step has a longer page if you get stuck.

| Step | What you do | Details |
| --- | --- | --- |
| 1 | Add the JitPack dependency and EasyOpenCV, sync Gradle | [Install](docs/Install.md) |
| 2 | Copy [`EasyOBJDSample`](samples/EasyOBJDSample.java) into TeamCode. Set `WEBCAM_NAME` to match Configure Robot | [Sample OpMode](docs/SampleOpMode.md) |
| 3 | Run **EasyOBJD Sample**. D-pad **up** widens HSV, **down** tightens. Telemetry is cluster X/Y in inches | Same loop as the first release |
| 4 | Optional: copy [`EasyOBJDUserConfig`](samples/EasyOBJDUserConfig.java) and pass `EasyOBJDUserConfig.create()` to keep HSV / camera inches | [Tuning](docs/Tuning.md) |
| 5 | Optional: **EasyOBJD Calibrate** for tape focal length and tilt | [Tuning](docs/Tuning.md#2-inches--easyobjd-calibrate) |

```
EasyOBJD.createPipeline()
        │
        ▼
EasyOBJD Sample  —  D-pad HSV, getClusters() X/Y
        │
        ▼
optional UserConfig / Calibrate  —  saved HSV and camera inches
```

Samples are **not** in the JitPack AAR.

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

## 2. Copy the sample

Copy [`samples/EasyOBJDSample.java`](samples/EasyOBJDSample.java) into TeamCode. Set `WEBCAM_NAME` to the name in Configure Robot (`Webcam 1` by default).

```java
EasyOBJDPipeline pipeline = EasyOBJD.createPipeline();
webcam.setPipeline(pipeline);
```

D-pad **up** / **down** widens / tightens HSV in that same OpMode.

To keep HSV and camera inches after you leave the OpMode, copy [`EasyOBJDUserConfig`](samples/EasyOBJDUserConfig.java) and switch to `EasyOBJD.createPipeline(EasyOBJDUserConfig.create())`. You do not edit the library.

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

```java
pipeline = EasyOBJD.createPipeline();
webcam.setPipeline(pipeline);

if (gamepad1.dpadUpWasPressed()) {
    pipeline.adjustHsvRange(1);
}
if (gamepad1.dpadDownWasPressed()) {
    pipeline.adjustHsvRange(-1);
}

List<ClusterInfo> clusters = pipeline.getClusters();
for (ClusterInfo cluster : clusters) {
    // cluster.x / cluster.y — inches right / forward of the lens
}
```

Optional field frame: `pipeline.setRobotPose(...)`. Full listing: [Sample OpMode](docs/SampleOpMode.md). Builder: [API](docs/API.md).

## 4. Tune (practice, not matches)

1. Run **EasyOBJD Sample**. D-pad **up** = wider HSV, **down** = tighter.
2. Optional: paste those HSV numbers into `EasyOBJDUserConfig` and pass `UserConfig.create()` so TeleOp/auto keep them.
3. Optional: **EasyOBJD Calibrate** for focal length and tilt.

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
Aditi Rao - #23918 Super Sigma Robotics

AI tools (Cursor, ChatGPT, OpenAI Codex in Cursor) were used as development assistants for code generation, debugging, documentation, and refinement. Architecture, requirements, testing, validation, and final implementation decisions were directed and reviewed by the author.
