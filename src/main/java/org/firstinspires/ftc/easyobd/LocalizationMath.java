package org.firstinspires.ftc.easyobd;

/**
 * Pinhole + tilt localization. Pure Java so unit tests can cover the
 * camera-to-floor and field-frame conversions without loading OpenCV.
 *
 * <p>Level camera frame after tilt correction: {@code x} right, {@code y} up,
 * {@code z} forward along the floor.
 */
public final class LocalizationMath {
    public static final int CALIBRATION_WIDTH = 640;

    private LocalizationMath() {}

    /**
     * Focal length in pixels at {@code frameWidth}. If {@code focalAt640} is
     * positive it is scaled from 640-wide; otherwise it is derived from
     * horizontal FOV.
     */
    public static double focalPx(double focalAt640, double horizontalFovDeg, int frameWidth) {
        if (frameWidth <= 0) {
            return Double.NaN;
        }
        if (focalAt640 > 0) {
            return focalAt640 * (frameWidth / (double) CALIBRATION_WIDTH);
        }
        return (frameWidth / 2.0) / Math.tan(Math.toRadians(horizontalFovDeg / 2.0));
    }

    /** Smallest accepted apparent radius for a ball at {@code maxRangeIn}. */
    public static double minRadiusPx(double minBallDiameterIn, double maxRangeIn,
                                    double focalAt640, double horizontalFovDeg, int frameWidth) {
        double focal = focalPx(focalAt640, horizontalFovDeg, frameWidth);
        return (minBallDiameterIn * focal) / (2.0 * maxRangeIn);
    }

    /** Apparent radius of a known-diameter ball at a given range. */
    public static double expectedRadiusPx(double ballDiameterIn, double rangeIn,
                                         double focalAt640, double horizontalFovDeg, int frameWidth) {
        if (rangeIn <= 1e-6) {
            return Double.POSITIVE_INFINITY;
        }
        double focal = focalPx(focalAt640, horizontalFovDeg, frameWidth);
        return (ballDiameterIn * focal) / (2.0 * rangeIn);
    }

    /**
     * Size-based pinhole: optical-axis depth from diameter vs radius, then
     * rotated by downward pitch into level axes.
     *
     * @return {@code {x, y, z}} in inches (right, up, forward)
     */
    public static double[] pixelToCamera(double px, double py, double radiusPx,
                                        int frameWidth, int frameHeight,
                                        double ballDiameterIn,
                                        double focalAt640, double horizontalFovDeg,
                                        double tiltDeg) {
        double focal = focalPx(focalAt640, horizontalFovDeg, frameWidth);
        double zCam = (ballDiameterIn * focal) / (radiusPx * 2.0);
        double dx = px - frameWidth / 2.0;
        double dy = py - frameHeight / 2.0;
        double xCam = zCam * dx / focal;
        double yUpCam = -zCam * dy / focal;
        return tiltToLevel(xCam, yUpCam, zCam, tiltDeg);
    }

    /**
     * Floor-plane intersection at ball-center height using camera height + pitch.
     * Does not use apparent size.
     *
     * @return {@code {x, y, z}} or {@code null} if the ray never hits the plane
     */
    public static double[] pixelToFloor(double px, double py,
                                       int frameWidth, int frameHeight,
                                       double cameraHeightIn, double ballDiameterIn,
                                       double focalAt640, double horizontalFovDeg,
                                       double tiltDeg) {
        double focal = focalPx(focalAt640, horizontalFovDeg, frameWidth);
        double dx = px - frameWidth / 2.0;
        double dy = py - frameHeight / 2.0;
        double[] dir = tiltToLevel(dx, -dy, focal, tiltDeg);
        double planeY = ballDiameterIn / 2.0 - cameraHeightIn;
        if (dir[1] >= -1e-6) {
            return null;
        }
        double t = planeY / dir[1];
        if (t <= 0) {
            return null;
        }
        return new double[] { t * dir[0], planeY, t * dir[2] };
    }

    /** Positive {@code tiltDeg} = optical axis pitched down. */
    public static double[] tiltToLevel(double xCam, double yUpCam, double zCam, double tiltDeg) {
        double tilt = Math.toRadians(tiltDeg);
        double cos = Math.cos(tilt);
        double sin = Math.sin(tilt);
        return new double[] {
                xCam,
                -zCam * sin + yUpCam * cos,
                zCam * cos + yUpCam * sin
        };
    }

    /**
     * Image Y (from top) of the horizon. Floor balls should appear at a
     * larger Y (lower in the frame) than this line.
     */
    public static double horizonImageY(double tiltDeg, double focalPx, int frameHeight) {
        double dy = -focalPx * Math.tan(Math.toRadians(tiltDeg));
        return frameHeight / 2.0 + dy;
    }

    /**
     * Camera (x right, z forward) → robot center {right, forward}, inches.
     */
    public static double[] cameraToRobot(double camX, double camZ,
                                        double forwardOfCenter, double rightOfCenter,
                                        double yawRad) {
        double robotForward = camZ * Math.cos(yawRad) + camX * Math.sin(yawRad) + forwardOfCenter;
        double robotRight = -camZ * Math.sin(yawRad) + camX * Math.cos(yawRad) + rightOfCenter;
        return new double[] { robotRight, robotForward };
    }

    /**
     * Camera (x right, z forward) → robot center → field {x, y}.
     * Same yaw / offset convention as typical Pedro / AprilTag field frames.
     */
    public static double[] cameraToField(double camX, double camZ,
                                        double poseX, double poseY, double heading,
                                        double forwardOfCenter, double rightOfCenter,
                                        double yawRad) {
        double[] robot = cameraToRobot(camX, camZ, forwardOfCenter, rightOfCenter, yawRad);
        double robotRight = robot[0];
        double robotForward = robot[1];
        double fieldX = poseX + robotForward * Math.cos(heading) + robotRight * Math.sin(heading);
        double fieldY = poseY + robotForward * Math.sin(heading) - robotRight * Math.cos(heading);
        return new double[] { fieldX, fieldY };
    }

    public static double planarRange(double camX, double camZ) {
        return Math.hypot(camX, camZ);
    }

    public static boolean inRange(double forwardZ, double maxRangeIn) {
        return forwardZ > 0 && forwardZ <= maxRangeIn;
    }
}
