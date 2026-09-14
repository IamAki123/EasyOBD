# Copy-in samples

These files are **not** inside the JitPack AAR. Copy them into TeamCode (`org.firstinspires.ftc.teamcode`).

| File | When |
| --- | --- |
| [EasyOBJDUserConfig.java](EasyOBJDUserConfig.java) | Always. HSV, ball size, camera, webcam name. |
| [EasyOBJDSample.java](EasyOBJDSample.java) | Match TeleOp. |
| [EasyOBJDTuner.java](EasyOBJDTuner.java) | Practice: D-pad wider / tighter HSV. |
| [EasyOBJDCalibrateSample.java](EasyOBJDCalibrateSample.java) | Practice: tape focal length and tilt. |

Do not run Tuner or Calibrate in a match. Point every OpMode at `EasyOBJDUserConfig.create()`.

[Tuning](../docs/Tuning.md) · [Sample OpMode](../docs/SampleOpMode.md)
