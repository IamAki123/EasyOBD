# How EasyOBJD works (simple version)

[README](../README.md) · [Full formulas](Math.md)

This is the same pipeline as [Math.md](Math.md), explained like you are new to vision. No formulas required.

## The one-sentence version

The camera sees a colored circle on the floor. EasyOBJD figures out **how far right** and **how far forward** that piece is, in inches, from the **lens**.

It does **not** drive the robot. It does **not** replace odometry. Your OpMode already opens the webcam; EasyOBJD only looks at each frame.

## What you get

For each group of pieces (a **cluster**):

- **X** — inches to the right of the camera
- **Y** — inches forward of the camera (along the floor)
- **How** it got those numbers (`FLOOR_PLANE` or `SIZE_BASED`)

That is `getClusters()`. `getBestClusterForIntake()` picks one (closest by default).

## A frame on the robot, in English

### 1. Keep only the color you care about

You set HSV with D-pad on the sample (or optional `EasyOBJDUserConfig`). Everything else becomes black. If this mask is wrong, **nothing later can be right**.

### 2. Clean the mask a little

Tiny speckles are opened away; holes in the ball are closed. Arena lights still win if they blow out the color.

### 3. Group neighboring “yes” patches

The image is a coarse grid. Touching occupied cells are one pile of game pieces — a cluster.

### 4. Find circles

A round blob is one ball. A peanut-shaped blob is split into two circles when the edges look like two arcs. If two balls look like **one** perfect circle, you still get one.

### 5. Turn a pixel into inches (two ways)

**Preferred:** the camera is high and pointed down. EasyOBJD shoots a ray through the pixel and asks where it hits the height of the ball center. That is **floor-plane**. It needs a real downward tilt.

**Fallback:** it knows the official ball diameter. A bigger circle in the image means a closer ball. That is **size-based**. Glare and partial balls throw this off.

`cluster.localization` tells you which one was used.

### 6. Optional: field coordinates

If you also tell it where the **robot** is (`setRobotPose`), it can rotate those camera inches onto the field. If you never call that, field X/Y stay “not a number.” Camera X/Y still work.

## What you will see on the Driver Station

| Telemetry | Meaning |
| --- | --- |
| **Clusters / Balls** | How many groups / circles this frame |
| **X / Y** | Inches right / forward of the **lens** |
| **FLOOR_PLANE vs SIZE_BASED** | Which ranging method |
| **Process ms** | How long the last frame took |
| **MASK overlay** | What the HSV filter kept (Tuner) |

A yellow blob on the preview is not the same as a trusted X/Y inside `maxRangeInches`.

## What EasyOBJD is not

- Not a full 3D tracker
- Not a replacement for dead wheels
- Not “undistort my fisheye for me”
- Not something you skip HSV tuning for

If the **mask** is wrong, use the Tuner. If the mask is good but **inches** are wrong, tape height/tilt and run Calibrate. That order is the whole reliability story.

---

Want the equations? [Math.md](Math.md). Ready to use it? [README](../README.md).
