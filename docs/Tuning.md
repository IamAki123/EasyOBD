# Tuning

[README](../README.md) · [Sample OpMode](SampleOpMode.md) · [Troubleshooting](Troubleshooting.md)

Tune in **practice**, not matches. Change **one** thing at a time. Geometry (height, tilt, focal, ball diameter) first if inches are wrong; HSV if the mask is wrong.

> ‼️ **Point every OpMode at `EasyOBJDUserConfig`.** Copying the file does nothing if your TeleOp still builds a raw `EasyOBJDConfig` with library defaults.

```
EasyOBJDUserConfig (TeamCode)
        HSV, ball size, camera, webcam
                │
                ▼
        Tuner (HSV)  →  Calibrate (focal / tilt)  →  Sample / match TeleOp
```

## 1. HSV — EasyOBJD Tuner

Driver Station → **EasyOBJD Tuner**. Preview starts on **MASK**.

| Button | Effect |
| --- | --- |
| D-pad **up** | Wider (more pixels count as the piece) |
| D-pad **down** | Tighter (fewer false positives) |
| **X** | Cycle overlay (`MASK` / `GRID` / `BALLS` / `DISTANCES` / `FULL`) |

When the mask is a clean silhouette of the piece (not the whole field, not a hollow ring):

1. Read `H_LOW, S_LOW, V_LOW` and `H_HIGH, S_HIGH, V_HIGH` from telemetry.
2. Paste them into `EasyOBJDUserConfig`.
3. Rebuild.

HSV is OpenCV 8-bit (**H 0–179**). Red wraps around 0/179 — use two ranges (`extraColorRange` inside `create()`).

## 2. Inches — EasyOBJD Calibrate

Assumes the mask already sees **one** ball.

1. Tape **camera height** (tiles → **lens**, not the housing).
2. Put one official-size ball on the floor, **image-centered**, at a known distance (default 31 in).
3. Read suggested `FOCAL_LENGTH_PIXELS_AT_640` and `CAMERA_TILT_DEGREES`.
4. Paste into `EasyOBJDUserConfig`.

If the ball is **not** vertically centered, use a phone inclinometer on the housing for tilt instead of the suggested number.

`BALL_DIAMETER_INCHES` must be the real piece. Size-based range scales with it. `MIN_BALL_DIAMETER_INCHES` is a hole / far-object cutoff (keep a bit larger than the official diameter).

## 3. Suggested order on the field

1. Tape lens height and tilt.
2. Tuner until MASK looks right.
3. Calibrate until telemetry Y matches the tape at 2–3 distances.
4. Sample TeleOp. If X/Y twitch while driving, set `smoothingAlpha` on the library config (optional; 0 = raw).

## What the library will not fix

| Symptom | Usually |
| --- | --- |
| Empty mask | HSV / lighting / glare |
| Good mask, Y 1.5× too far | Focal or FOV wrong (old 700 px guess) |
| Good mask, Y only wrong far away | Tilt / pitch, or no undistortion at the edges |
| Level camera, no floor X/Y | Floor-plane needs downward tilt |
| Two balls = one cluster | Fully merged silhouette |

[Simple explanation](MathButDumbed.md) · [Formulas](Math.md)
