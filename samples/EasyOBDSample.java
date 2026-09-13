package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.easyobd.ClusterInfo;
import org.firstinspires.ftc.easyobd.EasyOBD;
import org.firstinspires.ftc.easyobd.EasyOBDCreation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/**
 * Copy this file into TeamCode. Preview is the yellow mask; telemetry is
 * cluster count, ball count, and each cluster's camera X/Y (inches).
 *
 * <p>Gamepad 1: D-pad up widens HSV, D-pad down tightens.</p>
 */
@TeleOp(name = "EasyOBD Sample", group = "EasyOBD")
public class EasyOBDSample extends OpMode {
    public static final String WEBCAM_NAME = "Webcam 1";

    private OpenCvWebcam webcam;
    private EasyOBDCreation easyOBD;
    private volatile boolean cameraInitialized = false;

    @SuppressLint("DiscouragedApi")
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        easyOBD = EasyOBD.createPipeline();
        webcam.setPipeline(easyOBD);

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
            EasyOBDCreation.nudgeHsvRange(1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            EasyOBDCreation.nudgeHsvRange(-1);
        }

        if (!cameraInitialized) {
            telemetry.addLine("Camera starting...");
        }
        telemetry.addLine("D-pad UP widen HSV, DOWN tighten");
        telemetry.addData("HSV lower (H,S,V)", "%.0f, %.0f, %.0f",
                EasyOBDCreation.yellowLower.val[0],
                EasyOBDCreation.yellowLower.val[1],
                EasyOBDCreation.yellowLower.val[2]);
        telemetry.addData("HSV upper (H,S,V)", "%.0f, %.0f, %.0f",
                EasyOBDCreation.yellowUpper.val[0],
                EasyOBDCreation.yellowUpper.val[1],
                EasyOBDCreation.yellowUpper.val[2]);
        telemetry.addData("Clusters detected", easyOBD.getClusterCount());
        telemetry.addData("Balls detected", easyOBD.getBallCount());

        List<ClusterInfo> clusters = easyOBD.getClusters();
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
