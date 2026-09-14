# Sample OpMode

[README](../README.md) · [Tuning](Tuning.md) · [API](API.md)

Copy these **TeamCode** files together (package `org.firstinspires.ftc.teamcode`):

| File | Required? |
| --- | --- |
| [`EasyOBJDUserConfig.java`](../samples/EasyOBJDUserConfig.java) | Yes — HSV, ball size, camera, webcam name |
| [`EasyOBJDSample.java`](../samples/EasyOBJDSample.java) | Yes — match TeleOp |
| [`EasyOBJDTuner.java`](../samples/EasyOBJDTuner.java) | Practice only |
| [`EasyOBJDCalibrateSample.java`](../samples/EasyOBJDCalibrateSample.java) | Practice only |

The tuners are **not** in the JitPack AAR. Do not select Tuner or Calibrate during a match.

## `init()` (the only numbers you should edit live in UserConfig)

```java
pipeline = EasyOBJD.createPipeline(EasyOBJDUserConfig.create());
webcam.setPipeline(pipeline);
webcam.openCameraDeviceAsync(...); // startStreaming from UserConfig width/height
```

Do **not** paste HSV or ball diameter into every OpMode. Point TeleOp and auto at `EasyOBJDUserConfig.create()`.

## `loop()`

```java
// Optional field frame (Pedro / pinpoints / OTOS), inches + heading rad:
// pipeline.setRobotPose(pose.getX(), pose.getY(), pose.getHeading());

ClusterInfo intake = pipeline.getBestClusterForIntake();
if (intake != null) {
    // intake.x = inches right of the lens
    // intake.y = inches forward of the lens
}
```

Leave field conversion off until camera X/Y match tape. `USE_FIELD_FRAME` in the sample only **displays** field numbers.

## First-run checklist

1. `WEBCAM_NAME` matches Configure Robot.
2. Run **EasyOBJD Tuner**. D-pad up = wider HSV, down = tighter. Copy telemetry into `EasyOBJDUserConfig`.
3. Run **EasyOBJD Calibrate** with one ball at a known distance. Copy focal / tilt into `EasyOBJDUserConfig`.
4. Run **EasyOBJD Sample** and compare `Y` to a tape measure.

Full listings: [`samples/`](../samples/).
