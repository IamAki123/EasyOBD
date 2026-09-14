package org.firstinspires.ftc.easyobd;

/**
 * Tape-measure helpers for {@link EasyOBDConfig#focalLengthPixelsAt640} and
 * {@link EasyOBDConfig#cameraTiltDegrees}.
 *
 * <p>Place one official-size ball on the floor at a known distance, centered
 * in the image. Read the apparent radius from the sample / debug overlay,
 * then set:
 *
 * <pre>
 * config.focalLengthPixelsAt640 = EasyOBDCalibration.focalLengthAt640(radiusPx, distanceIn, frameWidth);
 * config.cameraTiltDegrees = EasyOBDCalibration.suggestedTiltDegrees(cameraHeightIn, distanceIn);
 * </pre>
 *
 * <p>Suggested tilt assumes the ball sits on the optical axis (image center).
 * Prefer a phone inclinometer on the camera housing when the ball is not
 * centered vertically. Checkerboard / AprilTag extrinsics can replace these
 * later; this helper stays tape-only.
 */
public final class EasyOBDCalibration {
    public static final double DEFAULT_BALL_DIAMETER_INCHES = 2.8;
    public static final int CALIBRATION_WIDTH = LocalizationMath.CALIBRATION_WIDTH;

    private EasyOBDCalibration() {}

    /**
     * Focal length at 640-wide from a measured apparent radius.
     * {@code FOCAL = (2 * radiusPx * distance) / diameter}, then scaled to 640.
     */
    public static double focalLengthAt640(double apparentRadiusPx, double distanceInches, int frameWidth) {
        return focalLengthAt640(apparentRadiusPx, distanceInches, frameWidth, DEFAULT_BALL_DIAMETER_INCHES);
    }

    public static double focalLengthAt640(double apparentRadiusPx, double distanceInches,
                                         int frameWidth, double ballDiameterInches) {
        if (apparentRadiusPx <= 0 || distanceInches <= 0 || frameWidth <= 0 || ballDiameterInches <= 0) {
            return Double.NaN;
        }
        double focalAtFrame = (apparentRadiusPx * 2.0 * distanceInches) / ballDiameterInches;
        return focalAtFrame * (CALIBRATION_WIDTH / (double) frameWidth);
    }

    /**
     * Downward pitch that puts a floor ball at {@code floorDistanceInches}
     * on the optical axis. At 19 in height and 31 in tape, this is about 30°.
     */
    public static double suggestedTiltDegrees(double cameraHeightInches, double floorDistanceInches) {
        return suggestedTiltDegrees(cameraHeightInches, floorDistanceInches, DEFAULT_BALL_DIAMETER_INCHES);
    }

    public static double suggestedTiltDegrees(double cameraHeightInches, double floorDistanceInches,
                                             double ballDiameterInches) {
        if (floorDistanceInches <= 1e-6) {
            return Double.NaN;
        }
        double drop = cameraHeightInches - ballDiameterInches / 2.0;
        return Math.toDegrees(Math.atan(drop / floorDistanceInches));
    }

    /** Predicted apparent radius at a known distance, for checking a focal guess. */
    public static double expectedRadiusPx(double distanceInches, double focalAt640, int frameWidth) {
        return LocalizationMath.expectedRadiusPx(
                DEFAULT_BALL_DIAMETER_INCHES, distanceInches, focalAt640, 70.4, frameWidth);
    }

    /**
     * Horizontal FOV implied by a 640-wide focal length. Useful when converting
     * a tape calibration back into {@link EasyOBDConfig#horizontalFovDegrees}.
     */
    public static double horizontalFovDegrees(double focalAt640) {
        if (focalAt640 <= 1e-6) {
            return Double.NaN;
        }
        return Math.toDegrees(2.0 * Math.atan((CALIBRATION_WIDTH / 2.0) / focalAt640));
    }
}
