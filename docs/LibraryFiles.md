# What each file does

[README](../README.md) · [API](API.md) · [Docs index](DocsInfo.md)

Java package is `org.firstinspires.ftc.easyobd` (older EasyOBD imports still compile).

## You call these

| File | Role |
| --- | --- |
| `EasyOBJD.java` | Factory: `createPipeline()` / `createPipeline(config)`. |
| `EasyOBJDPipeline.java` | EasyOpenCV pipeline: mask, grid, balls, localize, overlay, snapshots. |
| `EasyOBJDConfig.java` | Library tunables. Teams should fill these via `EasyOBJDUserConfig`, not by editing this class. |
| `EasyOBJDCalibration.java` | Tape helpers: focal length and suggested tilt. |
| `ClusterInfo.java` / `Ball.java` | Immutable published detections. |
| `LocalizationMethod.java` | `FLOOR_PLANE` vs `SIZE_BASED`. |
| `OverlayMode.java` | Preview: `FULL`, `MASK`, `GRID`, `BALLS`, `DISTANCES`. |
| `IntakeHeuristic.java` | Closest / leftmost / highest-confidence pick. |

## Internals (tested without a camera)

| File | Role |
| --- | --- |
| `LocalizationMath.java` | Pinhole, tilt, floor hit, robot/field. |
| `GridClustering.java` | Occupied-cell connected components + tightness. |

## Deprecated 0.1 / 0.2 names

`EasyOBD`, `EasyOBDPipeline`, `EasyOBDConfig`, `EasyOBDCalibration`, `EasyOBDCreation` forward to the EasyOBJD types.

## Copy-in (not in the JitPack AAR)

| File | Role |
| --- | --- |
| `samples/EasyOBJDUserConfig.java` | **Your** HSV, ball size, camera, webcam name. |
| `samples/EasyOBJDTuner.java` | D-pad HSV wider / tighter. |
| `samples/EasyOBJDSample.java` | Match TeleOp. |
| `samples/EasyOBJDCalibrateSample.java` | Tape focal / tilt. |

See [samples/README.md](../samples/README.md).
