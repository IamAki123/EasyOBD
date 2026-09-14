/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobd;

/**
 * Sample / docs numbers. Keep these aligned with
 * {@code samples/EasyOBJDUserConfig.java} and
 * {@code samples/EasyOBJDCalibrateSample.java}.
 *
 * <p>These are not OpenCV frames — they check that the published defaults
 * still produce the inches the README and Calibrate sample claim.
 */
final class KnownGeometries {
    private KnownGeometries() {}

    /** {@code EasyOBJDUserConfig.BALL_DIAMETER_INCHES} */
    static final double BALL_DIAMETER_IN = 2.8;
    /** {@code EasyOBJDUserConfig.MIN_BALL_DIAMETER_INCHES} */
    static final double MIN_BALL_DIAMETER_IN = 3.0;
    /** {@code EasyOBJDUserConfig.CAMERA_HEIGHT_INCHES} */
    static final double CAMERA_HEIGHT_IN = 19.0;
    /** {@code EasyOBJDUserConfig.CAMERA_TILT_DEGREES} */
    static final double CAMERA_TILT_DEG = 25.0;
    /** {@code EasyOBJDUserConfig.HORIZONTAL_FOV_DEGREES} */
    static final double HORIZONTAL_FOV_DEG = 70.4;
    /** {@code EasyOBJDUserConfig.MAX_RANGE_INCHES} */
    static final double MAX_RANGE_IN = 60.0;
    /** {@code EasyOBJDCalibrateSample.KNOWN_DISTANCE_INCHES} */
    static final double CALIBRATE_DISTANCE_IN = 31.0;

    static final int STREAM_WIDTH = 640;
    static final int STREAM_HEIGHT = 480;
    static final int PROCESS_WIDTH = 320;

    static final double IMAGE_CX = STREAM_WIDTH / 2.0;
    static final double IMAGE_CY = STREAM_HEIGHT / 2.0;

    static final double DEFAULT_FORWARD_OFFSET_IN = EasyOBJDConfig.DEFAULT_CAMERA_FORWARD_OF_CENTER;
    static final double DEFAULT_RIGHT_OFFSET_IN = EasyOBJDConfig.DEFAULT_CAMERA_RIGHT_OF_CENTER;
    static final double DEFAULT_YAW_DEG = EasyOBJDConfig.DEFAULT_CAMERA_YAW_DEGREES;

    /** HSV defaults from UserConfig (OpenCV 8-bit). */
    static final double H_LOW = 21;
    static final double S_LOW = 95;
    static final double V_LOW = 85;
    static final double H_HIGH = 35;
    static final double S_HIGH = 255;
    static final double V_HIGH = 255;
}
