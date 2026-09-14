# Troubleshooting

[README](../README.md) · [Tuning](Tuning.md) · [Install](Install.md)

## Gradle / sync

- Add JitPack to the **same** `repositories` block TeamCode actually uses (`build.dependencies.gradle` or `settings.gradle` if `FAIL_ON_PROJECT_REPOS`).
- Autocomplete missing: confirm `implementation 'com.github.IamAki123:EasyOBJD:1.0.0'` synced. First JitPack build of a new tag can take a minute.
- `JAVA_HOME` on Java 8: Android Gradle Plugin 8.7 needs JDK 11+. Use Android Studio’s Embedded JDK.

Building this repo (not TeamCode): [Contributing](../CONTRIBUTING.md).

## Camera does not open / crash on init

**Webcam name.** `WEBCAM_NAME` in the sample (or `EasyOBJDUserConfig` if you copied it) must match **Configure Robot** exactly (`Webcam 1`, `Webcam 2`, …). A mismatch throws on `hardwareMap.get`.

USB: Control Hub port, cable seated. Only one OpMode should own the camera.

## Nothing in the mask (all black)

1. Overlay `MASK` or `FULL`.
2. **EasyOBJD Sample**: D-pad **up** widens, **down** tightens.
3. Defaults are **yellow** (`21, 95, 85` … `35, 255, 255`, H 0–179). Other colors: `pipeline.getConfig().hsv(...)` or optional `EasyOBJDUserConfig`.
4. Red wraps around 0/179: two ranges via `extraColorRange`.
5. `ADAPTIVE_LIGHTING` helps dim arenas. Glare still punches holes — hood the lens.

## False positives

Tighten HSV (D-pad **down**). Raise `whiteRatioThreshold` on the library config if needed. `verticalBandFilter` drops detections above the horizon. Check `cluster.confidence` before driving.

## X/Y look wrong

1. Tape **lens** height and tilt (inclinometer). A **level** camera cannot floor-localize a ball on the optical axis.
2. **EasyOBJD Calibrate**: one on-axis ball at a known distance → `FOCAL_LENGTH_PIXELS_AT_640` and `CAMERA_TILT_DEGREES`.
3. `BALL_DIAMETER_INCHES` must be the real piece.
4. No undistortion — distrust the image edges.
5. `ClusterInfo.x` is **right**, `.y` is **forward**, from the **lens**.

Constant offset everywhere → geometry. Error only far away or at the edge → tilt / lens.

## Floor vs size

`localization == FLOOR_PLANE` when the ray hits the ball-center plane inside `maxRangeInches`. Else `SIZE_BASED`. All `SIZE_BASED` usually means tilt is too small.

## Two balls report as one

Fully merged silhouettes look like one circle. The splitter handles peanuts, not a single round blob.

## Jitter / slow

`smoothingAlpha` 0.25–0.40 for driving; `0` for raw. `PROCESS_WIDTH = 320` for cycle time. Leave Hough off unless you measured it.

## Field coordinates are NaN

You never called `setRobotPose`. Camera X/Y still work.

```java
pipeline.setRobotPose(pose.getX(), pose.getY(), pose.getHeading());
```

## Other season / other color

Edit HSV on the running pipeline (`adjustHsvRange`) or optional `EasyOBJDUserConfig` (`H_LOW`…`V_HIGH`, `BALL_DIAMETER_INCHES`).

## Tests vs the robot

JUnit covers pinhole/tilt math, grid grouping, and [sample default inches](../src/test/java/org/firstinspires/ftc/easyobd/SampleValuesTest.java). A green CI run does not prove arena lighting.
