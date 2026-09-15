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

import org.firstinspires.ftc.easyobjd.ClusterInfo;
import org.firstinspires.ftc.easyobjd.EasyOBJD;
import org.firstinspires.ftc.easyobjd.EasyOBJDPipeline;
import org.firstinspires.ftc.easyobjd.OverlayMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/**
 * Copy this file into TeamCode. Preview is the color mask; telemetry is
 * cluster count, ball count, and each cluster's camera X/Y (inches).
 *
 * <p>Gamepad 1: D-pad up widens HSV, D-pad down tightens.</p>
 *
 * <p>TeamCode will not resolve {@code org.firstinspires.ftc.easyobjd} until
 * JitPack is a repository and TeamCode depends on the library:</p>
 * <pre>
 * maven { url = 'https://jitpack.io' }
 * implementation 'org.openftc:easyopencv:1.7.3'
 * implementation 'com.github.IamAki123:EasyOBJD:1.0.0'
 * </pre>
 * Then File → Sync Project with Gradle Files.
 */
//noinspection SpellCheckingInspection
@SuppressWarnings("unused")
@TeleOp(name = "EasyOBJD Sample", group = "EasyOBJD")
public class EasyOBJDSample extends OpMode {
    public static final String WEBCAM_NAME = "Webcam 1";

    private OpenCvWebcam webcam;
    private EasyOBJDPipeline pipeline;
    private volatile boolean cameraInitialized = false;

    @SuppressLint("DiscouragedApi")
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        pipeline = EasyOBJD.createPipeline();
        pipeline.getConfig().overlayMode = OverlayMode.MASK;
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT,
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

        if (!cameraInitialized) {
            telemetry.addLine("Camera starting...");
        }
        telemetry.addLine("D-pad UP widen HSV, DOWN tighten");
        telemetry.addData("HSV lower (H,S,V)", "%.0f, %.0f, %.0f",
                pipeline.getConfig().hsvLower.val[0],
                pipeline.getConfig().hsvLower.val[1],
                pipeline.getConfig().hsvLower.val[2]);
        telemetry.addData("HSV upper (H,S,V)", "%.0f, %.0f, %.0f",
                pipeline.getConfig().hsvUpper.val[0],
                pipeline.getConfig().hsvUpper.val[1],
                pipeline.getConfig().hsvUpper.val[2]);
        telemetry.addData("Clusters detected", pipeline.getClusterCount());
        telemetry.addData("Balls detected", pipeline.getBallCount());

        List<ClusterInfo> clusters = pipeline.getClusters();
        for (ClusterInfo cluster : clusters) {
            telemetry.addLine("Cluster #" + cluster.id);
            telemetry.addData("#" + cluster.id + " X", "%.1f in", cluster.x);
            telemetry.addData("#" + cluster.id + " Y", "%.1f in", cluster.y);
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
