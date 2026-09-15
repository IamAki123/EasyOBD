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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class EasyOBJDCalibrationTest {

    @Test
    public void focalRoundTrip() {
        double distance = 31.0;
        double diameter = 2.8;
        double focalAt640 = 454.0;
        double radius = (diameter * focalAt640) / (2.0 * distance);
        double recovered = EasyOBJDCalibration.focalLengthAt640(radius, distance, 640, diameter);
        assertEquals(focalAt640, recovered, 0.01);
    }

    @Test
    public void focalScalesFrom320Frame() {
        double distance = 31.0;
        double diameter = 2.8;
        double focalAt640 = 454.0;
        double radius320 = (diameter * focalAt640 * 0.5) / (2.0 * distance);
        double recovered = EasyOBJDCalibration.focalLengthAt640(radius320, distance, 320, diameter);
        assertEquals(focalAt640, recovered, 0.01);
    }

    @Test
    public void suggestedTiltAbout30Degrees() {
        double tilt = EasyOBJDCalibration.suggestedTiltDegrees(19.0, 31.0);
        assertEquals(29.6, tilt, 1.0);
    }

    @Test
    public void fovFromFocalAgreesWithDefault() {
        double fov = EasyOBJDCalibration.horizontalFovDegrees(
                LocalizationMath.focalPx(0, 70.4, 640));
        assertEquals(70.4, fov, 0.05);
    }

    @Test
    public void invalidInputsAreNaN() {
        assertTrue(Double.isNaN(EasyOBJDCalibration.focalLengthAt640(0, 31, 640)));
        assertTrue(Double.isNaN(EasyOBJDCalibration.suggestedTiltDegrees(19, 0)));
    }
}
