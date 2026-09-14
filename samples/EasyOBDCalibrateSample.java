package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.easyobd.Ball;
import org.firstinspires.ftc.easyobd.EasyOBD;
import org.firstinspires.ftc.easyobd.EasyOBDCalibration;
import org.firstinspires.ftc.easyobd.EasyOBDConfig;
import org.firstinspires.ftc.easyobd.EasyOBDPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/**
 * Tape calibration helper. Copy into TeamCode.
 *
 * <ol>
 *   <li>Measure camera height above the floor (housing → tiles).</li>
 *   <li>Put one official ball on the floor on the optical axis (image
 *       center) at a known tape distance (e.g. 31 in from the lens).</li>
 *   <li>Read apparent radius from telemetry. Suggested focal and tilt
 *       update live.</li>
 *   <li>Put those numbers into {@link EasyOBDConfig} (or the sample
 *       {@code .camera(...)} / {@code .focalLengthPixelsAt640(...)} calls).</li>
 * </ol>
 *
 * <p>A phone inclinometer on the camera housing is more accurate for tilt
 * when the ball is not vertically centered. Checkerboard / AprilTag
 * extrinsics can replace this later.
 *
 * <p>Gamepad 1 D-pad up / down: HSV. X: cycle overlay.
 */
@TeleOp(name = "EasyOBD Calibrate", group = "EasyOBD")
public class EasyOBDCalibrateSample extends OpMode {
    public static final String WEBCAM_NAME = "Webcam 1";
    public static final int STREAM_WIDTH = 640;
    public static final int STREAM_HEIGHT = 480;
    public static final OpenCvCameraRotation STREAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

    /** Known floor distance from the lens to the ball, inches. */
    public static final double KNOWN_DISTANCE_INCHES = 31.0;
    /** Tape-measured height of the lens above the tiles. */
    public static final double CAMERA_HEIGHT_INCHES = 19.0;

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
        easyOBD = EasyOBD.createPipeline(EasyOBDConfig.builder()
                .camera(CAMERA_HEIGHT_INCHES, 25.0, 70.4)
                .build());
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

        telemetry.addLine("Center one ball at " + KNOWN_DISTANCE_INCHES + " in");
        telemetry.addData("Camera height", "%.1f in", CAMERA_HEIGHT_INCHES);
        telemetry.addData("Process ms", "%.1f", easyOBD.getLastProcessTimeMs());

        Ball largest = largestBall(easyOBD.getBalls());
        if (largest == null) {
            telemetry.addLine("No ball — widen HSV or check lighting");
        } else {
            int processWidth = easyOBD.getConfig().processWidth > 0
                    ? easyOBD.getConfig().processWidth
                    : STREAM_WIDTH;
            double focal = EasyOBDCalibration.focalLengthAt640(
                    largest.radiusPx, KNOWN_DISTANCE_INCHES, processWidth);
            double tilt = EasyOBDCalibration.suggestedTiltDegrees(
                    CAMERA_HEIGHT_INCHES, KNOWN_DISTANCE_INCHES);
            telemetry.addData("Apparent radius", "%.1f px", largest.radiusPx);
            telemetry.addData("Set focalLengthPixelsAt640", "%.1f", focal);
            telemetry.addData("Set cameraTiltDegrees (if on-axis)", "%.1f", tilt);
            telemetry.addData("Implied FOV", "%.1f deg", EasyOBDCalibration.horizontalFovDegrees(focal));
        }

        easyOBD.addTelemetry(telemetry, true);
        telemetry.update();
    }

    private static Ball largestBall(List<Ball> balls) {
        Ball best = null;
        for (Ball ball : balls) {
            if (best == null || ball.radiusPx > best.radiusPx) {
                best = ball;
            }
        }
        return best;
    }

    @Override
    public void stop() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }
}
