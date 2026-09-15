# What each file does

[README](../README.md) · [API](API.md) · [Docs index](DocsInfo.md)

Java package is `org.firstinspires.ftc.easyobjd`.

## You call these

| File | Role |
| --- | --- |
| `EasyOBJD.java` | Factory plus nested `LocalizationMethod` and `IntakeHeuristic`. |
| `OverlayMode.java` | Preview: `FULL`, `MASK`, `GRID`, `BALLS`, `DISTANCES`. |
| `EasyOBJDPipeline.java` | EasyOpenCV pipeline: mask, grid, balls, localize, overlay, snapshots. |
| `EasyOBJDConfig.java` | Library tunables. Optional TeamCode `EasyOBJDUserConfig` fills these; you can also use `createPipeline()` defaults. |
| `EasyOBJDCalibration.java` | Tape helpers: focal length and suggested tilt. |
| `ClusterInfo.java` | Immutable published detections, including nested `ClusterInfo.Ball`. |

## Internals (tested without a camera)

| File | Role |
| --- | --- |
| `LocalizationMath.java` | Pinhole, tilt, floor hit, robot/field, plus package-private grid grouping. |

## Copy-in (not in the JitPack AAR)

| File | Role |
| --- | --- |
| `samples/EasyOBJDSample.java` | First-run TeleOp: `createPipeline()`, D-pad HSV, cluster X/Y. |
| `samples/EasyOBJDUserConfig.java` | Optional saved HSV, ball size, camera, webcam name. |
| `samples/EasyOBJDTuner.java` | Optional MASK overlay while nudging HSV. |
| `samples/EasyOBJDCalibrateSample.java` | Optional tape focal / tilt. |

See [samples/README.md](../samples/README.md).
