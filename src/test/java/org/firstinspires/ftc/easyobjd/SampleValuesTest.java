/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobjd;

import org.junit.Test;

import static org.firstinspires.ftc.easyobjd.KnownGeometries.BALL_DIAMETER_IN;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.CALIBRATE_DISTANCE_IN;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.CAMERA_HEIGHT_IN;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.CAMERA_TILT_DEG;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.DEFAULT_FORWARD_OFFSET_IN;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.DEFAULT_RIGHT_OFFSET_IN;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.DEFAULT_YAW_DEG;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.H_HIGH;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.H_LOW;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.HORIZONTAL_FOV_DEG;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.IMAGE_CX;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.IMAGE_CY;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.MAX_RANGE_IN;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.MIN_BALL_DIAMETER_IN;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.PROCESS_WIDTH;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.S_HIGH;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.S_LOW;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.STREAM_HEIGHT;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.STREAM_WIDTH;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.V_HIGH;
import static org.firstinspires.ftc.easyobjd.KnownGeometries.V_LOW;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * If someone changes sample defaults, these fail until docs / Calibrate
 * claims are updated. No webcam required.
 */
public class SampleValuesTest {

    private static final double INCH = 0.35;

    @Test
    public void calibrateSampleRecovers31InWhenBallIsOnAxis() {
        double tilt = EasyOBJDCalibration.suggestedTiltDegrees(
                CAMERA_HEIGHT_IN, CALIBRATE_DISTANCE_IN, BALL_DIAMETER_IN);
        double[] hit = LocalizationMath.pixelToFloor(
                IMAGE_CX, IMAGE_CY, STREAM_WIDTH, STREAM_HEIGHT,
                CAMERA_HEIGHT_IN, BALL_DIAMETER_IN,
                0, HORIZONTAL_FOV_DEG, tilt);
        assertNotNull(hit);
        assertEquals("centered ball should be on the optical-axis X", 0, hit[0], INCH);
        assertEquals(CALIBRATE_DISTANCE_IN, hit[2], 0.6);
        assertEquals(29.6, tilt, 1.0);
    }

    @Test
    public void userConfig25DegTiltPutsCenteredBallAbout38InForward() {
        double[] hit = LocalizationMath.pixelToFloor(
                IMAGE_CX, IMAGE_CY, STREAM_WIDTH, STREAM_HEIGHT,
                CAMERA_HEIGHT_IN, BALL_DIAMETER_IN,
                0, HORIZONTAL_FOV_DEG, CAMERA_TILT_DEG);
        assertNotNull(hit);
        double expectedZ = (CAMERA_HEIGHT_IN - BALL_DIAMETER_IN / 2.0)
                / Math.tan(Math.toRadians(CAMERA_TILT_DEG));
        assertEquals(expectedZ, hit[2], INCH);
        assertTrue("sample 25° is shallower than the 31 in on-axis tilt",
                hit[2] > CALIBRATE_DISTANCE_IN);
    }

    @Test
    public void sizeBasedRangeAtCalibrateDistanceMatchesPinhole() {
        double focal = LocalizationMath.focalPx(0, HORIZONTAL_FOV_DEG, STREAM_WIDTH);
        double radius = (BALL_DIAMETER_IN * focal) / (2.0 * CALIBRATE_DISTANCE_IN);
        double[] cam = LocalizationMath.pixelToCamera(
                IMAGE_CX, IMAGE_CY, radius, STREAM_WIDTH, STREAM_HEIGHT,
                BALL_DIAMETER_IN, 0, HORIZONTAL_FOV_DEG, 0);
        assertEquals(0, cam[0], INCH);
        assertEquals(CALIBRATE_DISTANCE_IN, cam[2], 0.4);
    }

    @Test
    public void calibrateFocalRoundTripUsesSampleBallSize() {
        double focalAt640 = LocalizationMath.focalPx(0, HORIZONTAL_FOV_DEG, STREAM_WIDTH);
        double radius = (BALL_DIAMETER_IN * focalAt640) / (2.0 * CALIBRATE_DISTANCE_IN);
        double recovered = EasyOBJDCalibration.focalLengthAt640(
                radius, CALIBRATE_DISTANCE_IN, STREAM_WIDTH, BALL_DIAMETER_IN);
        assertEquals(focalAt640, recovered, 0.05);
    }

    @Test
    public void processWidth320HalvesFocalAndRadius() {
        double focal640 = LocalizationMath.focalPx(0, HORIZONTAL_FOV_DEG, STREAM_WIDTH);
        double focal320 = LocalizationMath.focalPx(0, HORIZONTAL_FOV_DEG, PROCESS_WIDTH);
        assertEquals(focal640 / 2.0, focal320, 1e-6);
        double r640 = LocalizationMath.expectedRadiusPx(
                BALL_DIAMETER_IN, CALIBRATE_DISTANCE_IN, 0, HORIZONTAL_FOV_DEG, STREAM_WIDTH);
        double r320 = LocalizationMath.expectedRadiusPx(
                BALL_DIAMETER_IN, CALIBRATE_DISTANCE_IN, 0, HORIZONTAL_FOV_DEG, PROCESS_WIDTH);
        assertEquals(r640 / 2.0, r320, 1e-6);
    }

    @Test
    public void sampleMaxRangeAcceptsCalibrateDistanceAndRejectsPastFiveFeet() {
        assertTrue(LocalizationMath.inRange(CALIBRATE_DISTANCE_IN, MAX_RANGE_IN));
        assertFalse(LocalizationMath.inRange(MAX_RANGE_IN + 1, MAX_RANGE_IN));
        double minR = LocalizationMath.minRadiusPx(
                MIN_BALL_DIAMETER_IN, MAX_RANGE_IN, 0, HORIZONTAL_FOV_DEG, STREAM_WIDTH);
        assertTrue(minR > 0);
        double expectedAtMax = LocalizationMath.expectedRadiusPx(
                MIN_BALL_DIAMETER_IN, MAX_RANGE_IN, 0, HORIZONTAL_FOV_DEG, STREAM_WIDTH);
        assertEquals(expectedAtMax, minR, 1e-9);
    }

    @Test
    public void sampleMountOffsetsShiftRobotFrame() {
        double[] robot = LocalizationMath.cameraToRobot(
                0, CALIBRATE_DISTANCE_IN,
                DEFAULT_FORWARD_OFFSET_IN, DEFAULT_RIGHT_OFFSET_IN,
                Math.toRadians(DEFAULT_YAW_DEG));
        assertTrue("yaw + forward offset should move the reading off the raw camera Z",
                Math.abs(robot[1] - CALIBRATE_DISTANCE_IN) > 0.2);
        double[] field = LocalizationMath.cameraToField(
                0, CALIBRATE_DISTANCE_IN, 0, 0, 0,
                DEFAULT_FORWARD_OFFSET_IN, DEFAULT_RIGHT_OFFSET_IN,
                Math.toRadians(DEFAULT_YAW_DEG));
        assertEquals(robot[1], field[0], 1e-9);
        assertEquals(-robot[0], field[1], 1e-9);
    }

    @Test
    public void levelSampleCameraCannotFloorLocalizeImageCenter() {
        assertNull(LocalizationMath.pixelToFloor(
                IMAGE_CX, IMAGE_CY, STREAM_WIDTH, STREAM_HEIGHT,
                CAMERA_HEIGHT_IN, BALL_DIAMETER_IN,
                0, HORIZONTAL_FOV_DEG, 0));
    }

    @Test
    public void sampleHsvIsValidOpenCv8BitAndYellowBand() {
        assertTrue(H_LOW >= 0 && H_HIGH <= 179);
        assertTrue(H_LOW < H_HIGH);
        assertTrue(S_LOW >= 0 && S_HIGH <= 255);
        assertTrue(V_LOW >= 0 && V_HIGH <= 255);
        assertTrue("default band should still be yellow-ish hue", H_LOW >= 15 && H_HIGH <= 45);
    }
}
