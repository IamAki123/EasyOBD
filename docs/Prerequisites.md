# Prerequisites

[README](../README.md) · [Install](Install.md) · [Troubleshooting](Troubleshooting.md)

EasyOBJD sits on top of **EasyOpenCV**. It does not open the camera, does not drive the robot, and does not replace odometry.

## What you need

- An FTC SDK **10.x or 11.x** Android Studio project
- [EasyOpenCV 1.7.x](https://github.com/OpenFTC/EasyOpenCV) on the same TeamCode module
- A USB webcam named in the Robot Controller **Configure Robot** screen
- A circular game piece of known diameter (default assumes ~2.8 in yellow pollen)

## What EasyOBJD does

Every camera frame, the pipeline:

1. Builds an HSV mask of your configured color
2. Groups touching yellow (or other-color) cells into clusters
3. Fits circular balls, splitting peanut-shaped blobs when it can
4. Converts each cluster to **inches right / forward of the lens**

Optional: you push a robot pose (`setRobotPose`) and it also fills field X/Y.

## What EasyOBJD does not do

- It does **not** open or close the webcam (your OpMode / EasyOpenCV does)
- It does **not** replace Pedro, pinpoints, or OTOS
- It does **not** undistort a fisheye / wide lens
- It does **not** guarantee inches under glare, occlusion, or a level camera

## Before you blame the library

1. The webcam preview works in a stock EasyOpenCV OpMode.
2. `WEBCAM_NAME` in `EasyOBJDUserConfig` matches the RC configuration exactly.
3. You can see the piece in the **MASK** overlay (run **EasyOBJD Tuner**).
4. Camera height and tilt are taped, not guessed.

If the mask is empty, it is HSV / lighting, not localization math. If the mask is good but X/Y are wrong, it is height, tilt, focal length, or ball diameter.

## Coordinates (read once)

| Quantity | Units / convention |
| --- | --- |
| Cluster `x` | Inches **right** of the lens |
| Cluster `y` | Inches **forward** of the lens (along the floor) |
| Robot X/Y | Inches right / forward of robot center (mount offsets + yaw) |
| Field X/Y | Inches after `setRobotPose` (Pedro-style heading, radians) |
| HSV hue | OpenCV 8-bit: **0–179** (not 0–360) |
| Camera tilt | Degrees, **positive = pitched down** |
| Ball size | Inches, official diameter |

Next: [Install](Install.md).
