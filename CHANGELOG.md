# Changelog

## 1.0.0

First EasyOBJD release. Project renamed from EasyOBD. Java package stays `org.firstinspires.ftc.easyobd`. JitPack: `com.github.IamAki123:EasyOBJD:1.0.0`.

### Added
- TeamCode copy-in [`EasyOBJDUserConfig`](samples/EasyOBJDUserConfig.java), [`EasyOBJDTuner`](samples/EasyOBJDTuner.java) (D-pad HSV), and Calibrate sample.
- Explicit library config for HSV (`.hsv(...)`) and ball size (`.ballSize(...)`).
- Color-agnostic HSV nudge.
- EasyATL-style docs (Install, Tuning, API, math, contributing, security), MIT file headers, issue templates, and CI.
- `KnownGeometries` / `SampleValuesTest` for the published sample defaults.

### Changed
- Primary types: `EasyOBJD`, `EasyOBJDPipeline`, `EasyOBJDConfig`, `EasyOBJDCalibration`.
- `EasyOBD*` names remain as deprecated aliases.
- Sample documents `WEBCAM_NAME` vs the Robot Controller configuration.

## 0.2

Config, API, and robustness pass. Detection math is the same idea (HSV → grid → arc split → floor or size) with clearer contracts and fewer shared statics.

### Added
- `EasyOBDConfig` builder / per-pipeline tunables (camera, HSV, grid, overlay, smoothing, extra color ranges).
- `EasyOBDCalibration` plus `samples/EasyOBDCalibrateSample.java` for tape focal-length and tilt.
- Richer `ClusterInfo` / top-level `Ball`: range, ball list, circularity, confidence, pixel radius, `LocalizationMethod`, tightness, field X/Y.
- `getBestClusterForIntake`, overlay modes, `adjustHsvRange`, `addTelemetry`, `getLastProcessTimeMs` / error helpers.
- Optional adaptive lighting, process width independent of preview, temporal smoothing, HoughCircles verification.
- JUnit tests for pinhole + tilt math, grid clustering, and calibration helpers.
- Expanded README (coordinate frames, calibration, performance, limitations).

### Changed
- Pipeline class is `EasyOBDPipeline`. `EasyOBDCreation` remains as a deprecated subclass.
- `EasyOBD.createPipeline()` returns `EasyOBDPipeline` and accepts an optional `EasyOBDConfig`.
- `getClusters()` / `getBalls()` publish immutable snapshots (copy-on-write).
- Camera statics are defaults only; live tuning should go through `pipeline.getConfig()`.
- Sample shows camera + field usage, debug telemetry, and configurable stream / process size.
- Library version `0.2`. EasyOpenCV remains `1.7.3` (1.7.x).

### Fixed
- Leftmost-cluster floor-vs-size flag, size-based range, and pixel radius are actually published.
- Small / extreme-aspect process frames no longer assume a 12×12 grid.
- Contour `Mat`s from hole-fill and ball find are released each frame.
