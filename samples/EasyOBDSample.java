package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.easyobd.ClusterInfo;
import org.firstinspires.ftc.easyobd.EasyOBD;
import org.firstinspires.ftc.easyobd.EasyOBDConfig;
import org.firstinspires.ftc.easyobd.EasyOBDPipeline;
import org.firstinspires.ftc.easyobd.IntakeHeuristic;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Copy this file into TeamCode. Preview is the yellow mask plus overlay;
 * telemetry is cluster count, camera X/Y, and (optionally) field X/Y.
 *
 * <p>Gamepad 1:
 * D-pad up / down widens / tightens HSV.
 * X cycles overlay (mask / grid / balls / distances / full).
 *
 * <p>To convert into field coordinates, call
 * {@code easyOBD.setRobotPose(pedroX, pedroY, headingRad)} every loop from
 * your localizer. Camera-frame X/Y do not need a pose.
 */
@TeleOp(name = "EasyOBD Sample", group = "EasyOBD")
public class EasyOBDSample extends OpMode {
    public static final String WEBCAM_NAME = "Webcam 1";

    /** Stream / preview size. Processing can be smaller via {@link #PROCESS_WIDTH}. */
    public static final int STREAM_WIDTH = 640;
    public static final int STREAM_HEIGHT = 480;
    public static final OpenCvCameraRotation STREAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

    /**
     * Process at 320-wide (height follows aspect) for Control Hub cycle time.
     * Set 0 to process at {@link #STREAM_WIDTH}. 640 is more accurate for
     * distant balls; 320 is typically 2–3× faster.
     */
    public static final int PROCESS_WIDTH = 320;

    /** Extra telemetry: process ms, occupied cells, errors, field pose. */
    public static final boolean DEBUG_MODE = true;

    /**
     * Set true after you wire {@link EasyOBDPipeline#setRobotPose}.
     * When false, only camera-frame X (right) / Y (forward) are shown.
     */
    public static final boolean USE_FIELD_FRAME = false;

    private OpenCvWebcam webcam;
    private EasyOBDPipeline easyOBD;
    private volatile boolean cameraInitialized = false;

    @SuppressLint("DiscouragedApi")
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);

        EasyOBDConfig config = EasyOBDConfig.builder()
                .camera(19.0, 25.0, 70.4)
                .processWidth(PROCESS_WIDTH)
                .adaptiveLighting(true)
                .build();
        easyOBD = EasyOBD.createPipeline(config);
        webcam.setPipeline(easyOBD);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, STREAM_ROTATION,
                        OpenCvWebcam.StreamFormat.MJPEG);
                cameraInitialized = true;
            }

            @Override
            public void onError(int errorCode) {
                cameraInitialized = false;
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    @Override
    public void init_loop() {
        tick();
    }

    @Override
    public void loop() {
        // If you have Pedro / pinpoints / SparkFun OTOS:
        // easyOBD.setRobotPose(localizer.getX(), localizer.getY(), localizer.getHeading());
        tick();
    }

    private void tick() {
        if (gamepad1.dpadUpWasPressed()) {
            easyOBD.adjustHsvRange(1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            easyOBD.adjustHsvRange(-1);
        }
        if (gamepad1.xWasPressed()) {
            easyOBD.cycleOverlayMode();
        }

        if (!cameraInitialized) {
            telemetry.addLine("Camera starting...");
        }
        telemetry.addLine("D-pad UP/DOWN HSV   X cycle overlay");
        telemetry.addData("HSV lower (H,S,V)", "%.0f, %.0f, %.0f",
                easyOBD.getConfig().yellowLower.val[0],
                easyOBD.getConfig().yellowLower.val[1],
                easyOBD.getConfig().yellowLower.val[2]);
        telemetry.addData("HSV upper (H,S,V)", "%.0f, %.0f, %.0f",
                easyOBD.getConfig().yellowUpper.val[0],
                easyOBD.getConfig().yellowUpper.val[1],
                easyOBD.getConfig().yellowUpper.val[2]);

        easyOBD.addTelemetry(telemetry, DEBUG_MODE);

        ClusterInfo intake = easyOBD.getBestClusterForIntake(IntakeHeuristic.CLOSEST);
        if (intake != null) {
            telemetry.addData("Intake target", "#%d  Y=%.1f in", intake.id, intake.y);
        }

        if (USE_FIELD_FRAME) {
            telemetry.addData("Field X", "%.1f", easyOBD.FieldX.bestClusterCenterpoint());
            telemetry.addData("Field Y", "%.1f", easyOBD.FieldY.bestClusterCenterpoint());
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }
}
