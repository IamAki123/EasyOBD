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

import org.firstinspires.ftc.easyobd.EasyOBJD;
import org.firstinspires.ftc.easyobd.EasyOBJDPipeline;
import org.firstinspires.ftc.easyobd.EasyOBJD.OverlayMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Copy into TeamCode with {@link EasyOBJDUserConfig}.
 *
 * <p>Starts from your user-config HSV, then:
 * <ul>
 *   <li>D-pad <b>up</b> — wider (more pixels count as the piece)</li>
 *   <li>D-pad <b>down</b> — tighter (fewer false positives)</li>
 *   <li>X — cycle overlay (start on MASK so you see the color cutout)</li>
 * </ul>
 *
 * <p>When the mask looks right, copy the HSV numbers from telemetry into
 * {@link EasyOBJDUserConfig} ({@code H_LOW} … {@code V_HIGH}) and rebuild.
 */
@TeleOp(name = "EasyOBJD Tuner", group = "EasyOBJD")
public class EasyOBJDTuner extends OpMode {
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
        pipeline.getConfig().overlayMode = OverlayMode.MASK;
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
        if (gamepad1.dpadUpWasPressed()) {
            pipeline.adjustHsvRange(1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            pipeline.adjustHsvRange(-1);
        }
        if (gamepad1.xWasPressed()) {
            pipeline.cycleOverlayMode();
        }

        if (!cameraInitialized) {
            telemetry.addLine("Camera starting...");
        }

        telemetry.addLine("D-pad UP = wider HSV");
        telemetry.addLine("D-pad DOWN = tighter HSV");
        telemetry.addLine("X = cycle overlay (MASK recommended)");
        telemetry.addLine("Copy these into EasyOBJDUserConfig:");
        telemetry.addData("H_LOW, S_LOW, V_LOW", "%.0f, %.0f, %.0f",
                pipeline.getConfig().hsvLower.val[0],
                pipeline.getConfig().hsvLower.val[1],
                pipeline.getConfig().hsvLower.val[2]);
        telemetry.addData("H_HIGH, S_HIGH, V_HIGH", "%.0f, %.0f, %.0f",
                pipeline.getConfig().hsvUpper.val[0],
                pipeline.getConfig().hsvUpper.val[1],
                pipeline.getConfig().hsvUpper.val[2]);
        telemetry.addData("Ball diameter (from user config)", "%.2f in",
                EasyOBJDUserConfig.BALL_DIAMETER_INCHES);
        telemetry.addData("Overlay", pipeline.getConfig().overlayMode);
        telemetry.addData("Process ms", "%.1f", pipeline.getLastProcessTimeMs());
        if (pipeline.getLastError() != null) {
            telemetry.addData("Error", pipeline.getLastError());
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
