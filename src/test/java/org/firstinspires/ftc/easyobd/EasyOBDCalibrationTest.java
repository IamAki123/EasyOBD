package org.firstinspires.ftc.easyobd;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class EasyOBDCalibrationTest {

    @Test
    public void focalRoundTrip() {
        double distance = 31.0;
        double diameter = 2.8;
        double focalAt640 = 454.0;
        double radius = (diameter * focalAt640) / (2.0 * distance);
        double recovered = EasyOBDCalibration.focalLengthAt640(radius, distance, 640, diameter);
        assertEquals(focalAt640, recovered, 0.01);
    }

    @Test
    public void focalScalesFrom320Frame() {
        double distance = 31.0;
        double diameter = 2.8;
        double focalAt640 = 454.0;
        double radius320 = (diameter * focalAt640 * 0.5) / (2.0 * distance);
        double recovered = EasyOBDCalibration.focalLengthAt640(radius320, distance, 320, diameter);
        assertEquals(focalAt640, recovered, 0.01);
    }

    @Test
    public void suggestedTiltAbout30Degrees() {
        double tilt = EasyOBDCalibration.suggestedTiltDegrees(19.0, 31.0);
        assertEquals(29.6, tilt, 1.0);
    }

    @Test
    public void fovFromFocalAgreesWithDefault() {
        double fov = EasyOBDCalibration.horizontalFovDegrees(
                LocalizationMath.focalPx(0, 70.4, 640));
        assertEquals(70.4, fov, 0.05);
    }

    @Test
    public void invalidInputsAreNaN() {
        assertTrue(Double.isNaN(EasyOBDCalibration.focalLengthAt640(0, 31, 640)));
        assertTrue(Double.isNaN(EasyOBDCalibration.suggestedTiltDegrees(19, 0)));
    }
}
