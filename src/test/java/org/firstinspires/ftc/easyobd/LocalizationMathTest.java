/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobd;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class LocalizationMathTest {

    private static final double EPS = 0.15;

    @Test
    public void focalFromFovAt640Matches70Point4() {
        double focal = LocalizationMath.focalPx(0, 70.4, 640);
        assertEquals(454.0, focal, 2.0);
    }

    @Test
    public void focalAt640ScalesWithWidth() {
        double at640 = LocalizationMath.focalPx(400, 70.4, 640);
        double at320 = LocalizationMath.focalPx(400, 70.4, 320);
        assertEquals(at640 / 2.0, at320, 1e-9);
    }

    @Test
    public void tiltIdentityWhenLevel() {
        double[] level = LocalizationMath.tiltToLevel(3, 4, 5, 0);
        assertEquals(3, level[0], 1e-9);
        assertEquals(4, level[1], 1e-9);
        assertEquals(5, level[2], 1e-9);
    }

    @Test
    public void floorHitAtImageCenterMatchesSuggestedTilt() {
        double height = 19.0;
        double distance = 31.0;
        double diameter = 2.8;
        double tilt = EasyOBJDCalibration.suggestedTiltDegrees(height, distance, diameter);
        double[] hit = LocalizationMath.pixelToFloor(
                320, 240, 640, 480,
                height, diameter, 0, 70.4, tilt);
        assertNotNull(hit);
        assertEquals(0, hit[0], EPS);
        assertEquals(diameter / 2.0 - height, hit[1], EPS);
        assertEquals(distance, hit[2], 0.6);
    }

    @Test
    public void levelCameraMissesNearbyFloorBallAtImageCenter() {
        double[] hit = LocalizationMath.pixelToFloor(
                320, 240, 640, 480,
                19.0, 2.8, 0, 70.4, 0);
        assertNull(hit);
    }

    @Test
    public void sizeBasedRangeMatchesPinhole() {
        double diameter = 2.8;
        double focal = LocalizationMath.focalPx(0, 70.4, 640);
        double range = 40.0;
        double radius = (diameter * focal) / (2.0 * range);
        double[] cam = LocalizationMath.pixelToCamera(
                320, 240, radius, 640, 480, diameter, 0, 70.4, 0);
        assertEquals(0, cam[0], EPS);
        assertEquals(range, cam[2], 0.3);
    }

    @Test
    public void cameraToFieldHeadingZero() {
        double[] field = LocalizationMath.cameraToField(
                4, 10, 100, 50, 0, 0, 0, 0);
        assertEquals(110, field[0], EPS);
        assertEquals(46, field[1], EPS);
    }

    @Test
    public void horizonIsAboveCenterWhenPitchedDown() {
        double focal = LocalizationMath.focalPx(0, 70.4, 640);
        double y = LocalizationMath.horizonImageY(25, focal, 480);
        assertTrue(y < 240);
        assertTrue(y > 0);
    }

    @Test
    public void inRangeRejectsBehindAndTooFar() {
        assertTrue(LocalizationMath.inRange(20, 60));
        assertTrue(!LocalizationMath.inRange(0, 60));
        assertTrue(!LocalizationMath.inRange(61, 60));
    }
}
