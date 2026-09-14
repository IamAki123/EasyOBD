# API

[README](../README.md) · [Library files](LibraryFiles.md) · [Sample OpMode](SampleOpMode.md)

Package: `org.firstinspires.ftc.easyobd`. Current JitPack: **1.0.0**.

## EasyOBJD

```java
EasyOBJDPipeline createPipeline()
EasyOBJDPipeline createPipeline(EasyOBJDConfig config)
```

Nested enums (import as `EasyOBJD.OverlayMode`, etc.):

| Enum | Values |
| --- | --- |
| `OverlayMode` | `FULL`, `MASK`, `GRID`, `BALLS`, `DISTANCES` |
| `LocalizationMethod` | `FLOOR_PLANE`, `SIZE_BASED`, `NONE` |
| `IntakeHeuristic` | `CLOSEST`, `LEFTMOST`, `HIGHEST_CONFIDENCE` |

Start with `EasyOBJD.createPipeline()` (library defaults). Pass `EasyOBJDUserConfig.create()` only if you copied that TeamCode file.

## EasyOBJDPipeline

Constructor: `new EasyOBJDPipeline()` (copies static defaults) or `new EasyOBJDPipeline(config)`.

| Method | Meaning |
| --- | --- |
| `getConfig()` | Live-tunable instance config (HSV, ball size, overlay). |
| `setConfig(EasyOBJDConfig)` | Replace fields on the running config. |
| `adjustHsvRange(int delta)` | Wider (`+1`) or tighter (`-1`). Color-agnostic. |
| `cycleOverlayMode()` | `MASK` → `GRID` → `BALLS` → `DISTANCES` → `FULL`. |
| `setRobotPose(x, y, headingRad)` | Inches + radians; enables field X/Y. |
| `getClusters()` | Immutable list, left-to-right. |
| `getBalls()` | Immutable circular pieces this frame. |
| `getBestClusterForIntake()` | Closest planar range. |
| `getBestClusterForIntake(IntakeHeuristic)` | `CLOSEST`, `LEFTMOST`, `HIGHEST_CONFIDENCE`. |
| `getClusterX()` / `getClusterY()` | Leftmost cluster, inches right / forward of the lens. |
| `getClusterRobotX()` / `getClusterRobotY()` | Same, robot center (offsets + yaw). |
| `usedFloorPlane()` | Leftmost cluster used floor-plane. |
| `addTelemetry(telemetry)` / `addTelemetry(telemetry, debug)` | Counts, X/Y, optional process ms / errors. |
| `getLastProcessTimeMs()` / `getLastError()` / `getFrameCount()` | Debug. |
| `FieldX` / `FieldY` | `.bestClusterCenterpoint()` after a pose is set; else NaN. |

`processFrame` is called by EasyOpenCV. Do not call it from the OpMode loop.

Statics such as `CAMERA_HEIGHT_INCHES` are **defaults only**. After construction, change `getConfig()` or pass `EasyOBJDUserConfig.create()`.

## ClusterInfo

| Field | Meaning |
| --- | --- |
| `id` | 1…n, left to right |
| `x`, `y` | Inches right / forward of the lens |
| `rangeInches` | `hypot(x, y)` |
| `localization` | `FLOOR_PLANE` or `SIZE_BASED` |
| `balls` / `ballCount` | Associated circles |
| `confidence`, `circularity`, `pixelRadius`, `tightness` | Quality hints |
| `fieldX`, `fieldY` | After `setRobotPose`; else NaN |
| `centerPx`, `cells` | Image centroid and occupied grid cells |

## ClusterInfo.Ball

`center`, `radiusPx`, `area`, `circularity`, `x/y/z` (size-based camera frame), `confidence`. Always `SIZE_BASED`. Import `org.firstinspires.ftc.easyobd.ClusterInfo.Ball`.

## EasyOBJDConfig (library)

Builder highlights: `camera`, `hsv`, `ballSize` / `ballDiameterInches`, `processWidth`, `maxRangeInches`, `adaptiveLighting`, `smoothingAlpha`, `overlayMode`, `extraColorRange`.

Teams should edit [`EasyOBJDUserConfig`](../samples/EasyOBJDUserConfig.java), which calls this builder.

## EasyOBJDCalibration

```java
focalLengthAt640(radiusPx, distanceIn, frameWidth)
focalLengthAt640(radiusPx, distanceIn, frameWidth, ballDiameterIn)
suggestedTiltDegrees(cameraHeightIn, floorDistanceIn)
suggestedTiltDegrees(cameraHeightIn, floorDistanceIn, ballDiameterIn)
horizontalFovDegrees(focalAt640)
```

