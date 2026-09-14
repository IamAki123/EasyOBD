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

import org.firstinspires.ftc.easyobd.ClusterInfo;
import org.firstinspires.ftc.easyobd.EasyOBJD;
import org.firstinspires.ftc.easyobd.EasyOBJDPipeline;
import org.firstinspires.ftc.easyobd.IntakeHeuristic;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Copy this file <b>and</b> {@link EasyOBJDUserConfig} into TeamCode.
 * Edit HSV, ball size, camera, and webcam name in {@link EasyOBJDUserConfig}
 * — not in the library.
 *
 * <p>For live HSV (wider / tighter), run <b>EasyOBJD Tuner</b>, then paste
 * the telemetry numbers back into {@link EasyOBJDUserConfig}.
 *
 * <p>Gamepad 1 X cycles overlay. Field coords need
 * {@code pipeline.setRobotPose(...)} every loop (see loop()).
 */
@TeleOp(name = "EasyOBJD Sample", group = "EasyOBJD")
public class EasyOBJDSample extends OpMode {
    public static final boolean DEBUG_MODE = true;
    public static final boolean USE_FIELD_FRAME = false;

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
        // If you have Pedro / pinpoints / SparkFun OTOS:
        // pipeline.setRobotPose(localizer.getX(), localizer.getY(), localizer.getHeading());
        tick();
    }

    private void tick() {
        if (gamepad1.xWasPressed()) {
            pipeline.cycleOverlayMode();
        }

        if (!cameraInitialized) {
            telemetry.addLine("Camera starting...");
        }
        telemetry.addLine("HSV / ball size: edit EasyOBJDUserConfig");
        telemetry.addLine("Live HSV: run EasyOBJD Tuner (D-pad up/down)");
        telemetry.addData("HSV lower", "%.0f, %.0f, %.0f",
                pipeline.getConfig().hsvLower.val[0],
                pipeline.getConfig().hsvLower.val[1],
                pipeline.getConfig().hsvLower.val[2]);
        telemetry.addData("Ball diameter", "%.2f in", EasyOBJDUserConfig.BALL_DIAMETER_INCHES);

        pipeline.addTelemetry(telemetry, DEBUG_MODE);

        ClusterInfo intake = pipeline.getBestClusterForIntake(IntakeHeuristic.CLOSEST);
        if (intake != null) {
            telemetry.addData("Intake target", "#%d  Y=%.1f in", intake.id, intake.y);
        }
        if (USE_FIELD_FRAME) {
            telemetry.addData("Field X", "%.1f", pipeline.FieldX.bestClusterCenterpoint());
            telemetry.addData("Field Y", "%.1f", pipeline.FieldY.bestClusterCenterpoint());
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
