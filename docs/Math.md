# How EasyOBJD works (formulas)

[README](../README.md) · [Simple version](MathButDumbed.md) · [API](API.md)

EasyOBJD finds circular game pieces and reports each cluster in **inches**. Defaults assume yellow ~2.8 in balls, a 12×12 grid, and a camera about 19 in high with ~25° downward tilt. Change those in **`EasyOBJDUserConfig`**, not by editing the AAR.

## Pipeline

```
RGB/RGBA frame
    → optional downscale (processWidth / processScale)
    → blur → HSV
    → inRange(hsvLower, hsvUpper)  [OR extraColorRanges]
    → open / close (kernel scales with width) → fill holes
    → integral image + grid → connected occupied cells = clusters
    → contours → circularity / Kasa arc-split → balls
    → localize each cluster (floor-plane, else size-based)
    → optional EMA smooth → publish immutable snapshot
```

## Coordinate frames

Level camera after tilt correction:

| Axis | Meaning |
| --- | --- |
| `x` | right of the lens (inches) |
| `y` | up (inches); floor-plane hits have `y = ballRadius − cameraHeight` |
| `z` | forward along the floor (inches) |

`ClusterInfo.x` / `.y` are camera **right** and **forward** (`z`).

## Intrinsics

```
if focalLengthPixelsAt640 > 0:
    f = focalLengthPixelsAt640 * (frameWidth / 640)
else:
    f = (frameWidth / 2) / tan(horizontalFovDeg / 2)
```

Default FOV 70.4° at 640-wide ⇒ **f ≈ 454 px**.

```
f_frame = (2 * radiusPx * distance) / ballDiameter
f_640   = f_frame * (640 / frameWidth)
```

There is **no undistortion**. Wide lenses bias range near the edges.

## Size-based range (fallback)

```
Z_cam = (D * f) / (2 * r)
X_cam = Z_cam * (u − cx) / f
Y_up  = −Z_cam * (v − cy) / f

x = X_cam
y = −Z_cam sin τ + Y_up cos τ
z =  Z_cam cos τ + Y_up sin τ
```

Positive `τ` = optical axis pitched **down**.

## Floor-plane (preferred)

```
dir = tiltToLevel(u − cx, −(v − cy), f, τ)
t   = planeY / dir.y     // only if dir.y < 0
hit = t * dir
```

`planeY = D/2 − cameraHeight`. If `hit.z` is in `(0, maxRangeInches]`, that hit is `FLOOR_PLANE`. A level camera cannot hit a floor ball on the optical axis.

```
τ = atan((cameraHeight − D/2) / floorDistance)
```

At 19 in height and 31 in tape that is about **30°**.

## Camera → robot → field

```
robotForward = z cos ψ + x sin ψ + forwardOffset
robotRight   = −z sin ψ + x cos ψ + rightOffset

fieldX = poseX + robotForward cos θ + robotRight sin θ
fieldY = poseY + robotForward sin θ − robotRight cos θ
```

## Grid and balls

Integral-image 12×12 (clamped on tiny frames). High-circularity contours skip arc-split. Kasa windows vote; weak RMS is dropped. Disk fill rejects the valley between two balls. Fully merged circles can still report as one.

Implemented in `LocalizationMath` (unit-tested) and `EasyOBJDPipeline`.
