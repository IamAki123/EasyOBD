/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.easyobd.ClusterInfo.Ball;
import org.firstinspires.ftc.easyobd.EasyOBJD;
import org.firstinspires.ftc.easyobd.EasyOBJDCalibration;
import org.firstinspires.ftc.easyobd.EasyOBJDPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/**
 * Tape calibration. Copy with {@link EasyOBJDUserConfig}.
 * HSV and ball size come from the user config; this OpMode suggests
 * {@code FOCAL_LENGTH_PIXELS_AT_640} and {@code CAMERA_TILT_DEGREES}
 * to paste back into that file.
 *
 * <p>For HSV wider/tighter, use <b>EasyOBJD Tuner</b> instead.
 */
@TeleOp(name = "EasyOBJD Calibrate", group = "EasyOBJD")
public class EasyOBJDCalibrateSample extends OpMode {
    /** Known floor distance from the lens to the ball, inches. */
    public static final double KNOWN_DISTANCE_INCHES = 31.0;

    private OpenCvWebcam webcam;
    private EasyOBJDPipeline pipeline;
    private volatile boolean cameraInitialized = false;

    @SuppressLint("DiscouragedApi")
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, EasyOBJDUserConfig.WEBCAM_NAME),
                cameraMonitorViewId);
        pipeline = EasyOBJD.createPipeline(EasyOBJDUserConfig.create());
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(
                        EasyOBJDUserConfig.STREAM_WIDTH,
                        EasyOBJDUserConfig.STREAM_HEIGHT,
                        EasyOBJDUserConfig.STREAM_ROTATION,
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
        if (gamepad1.xWasPressed()) {
            pipeline.cycleOverlayMode();
        }

        if (!cameraInitialized) {
            telemetry.addLine("Camera starting...");
        }

        telemetry.addLine("Center one ball at " + KNOWN_DISTANCE_INCHES + " in");
        telemetry.addLine("Copy suggested values into EasyOBJDUserConfig");
        telemetry.addData("Camera height (user config)", "%.1f in",
                EasyOBJDUserConfig.CAMERA_HEIGHT_INCHES);

        Ball largest = largestBall(pipeline.getBalls());
        if (largest == null) {
            telemetry.addLine("No ball — run EasyOBJD Tuner or widen HSV in UserConfig");
        } else {
            int processWidth = pipeline.getConfig().processWidth > 0
                    ? pipeline.getConfig().processWidth
                    : EasyOBJDUserConfig.STREAM_WIDTH;
            double focal = EasyOBJDCalibration.focalLengthAt640(
                    largest.radiusPx, KNOWN_DISTANCE_INCHES, processWidth,
                    EasyOBJDUserConfig.BALL_DIAMETER_INCHES);
            double tilt = EasyOBJDCalibration.suggestedTiltDegrees(
                    EasyOBJDUserConfig.CAMERA_HEIGHT_INCHES, KNOWN_DISTANCE_INCHES,
                    EasyOBJDUserConfig.BALL_DIAMETER_INCHES);
            telemetry.addData("Apparent radius", "%.1f px", largest.radiusPx);
            telemetry.addData("FOCAL_LENGTH_PIXELS_AT_640", "%.1f", focal);
            telemetry.addData("CAMERA_TILT_DEGREES (if on-axis)", "%.1f", tilt);
        }

        pipeline.addTelemetry(telemetry, true);
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
