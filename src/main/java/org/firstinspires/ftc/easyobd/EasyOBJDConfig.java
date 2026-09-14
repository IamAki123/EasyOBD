/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobd;

import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

/**
 * Per-pipeline tunables. Construct with {@link #builder()} or mutate fields
 * after {@link EasyOBJDPipeline#getConfig()} for live TeleOp adjustments.
 *
 * <p>Static fields on {@link EasyOBJDPipeline} are <em>defaults only</em> —
 * copied when a pipeline is constructed with the no-arg constructor. Changing
 * those statics after construction does not affect existing pipelines.
 */
public class EasyOBJDConfig {

    public static final double DEFAULT_BALL_DIAMETER_INCHES = EasyOBJDCalibration.DEFAULT_BALL_DIAMETER_INCHES;
    public static final double DEFAULT_CAMERA_HEIGHT_INCHES = 19.0;
    public static final double DEFAULT_CAMERA_TILT_DEGREES = 25.0;
    public static final double DEFAULT_HORIZONTAL_FOV_DEGREES = 70.4;
    public static final double DEFAULT_FOCAL_LENGTH_PIXELS_AT_640 = 0;
    public static final double DEFAULT_CAMERA_FORWARD_OF_CENTER = 0.75;
    public static final double DEFAULT_CAMERA_RIGHT_OF_CENTER = 0.25;
    public static final double DEFAULT_CAMERA_YAW_DEGREES = -4.0;
    public static final int DEFAULT_GRID_ROWS = 12;
    public static final int DEFAULT_GRID_COLS = 12;
    public static final double DEFAULT_WHITE_RATIO_THRESHOLD = 0.12;
    public static final double DEFAULT_BALL_SEARCH_RADIUS_PX = 220;
    public static final double DEFAULT_MIN_BALL_DIAMETER_INCHES = 3.0;
    public static final double DEFAULT_MAX_RANGE_INCHES = 60.0;
    public static final double DEFAULT_PROCESS_SCALE = 1.0;
    public static final double DEFAULT_H_LOWER = 21;
    public static final double DEFAULT_S_LOWER = 95;
    public static final double DEFAULT_V_LOWER = 85;
    public static final double DEFAULT_H_UPPER = 35;
    public static final double DEFAULT_S_UPPER = 255;
    public static final double DEFAULT_V_UPPER = 255;

    /** Official pollen / ball diameter used for size-based range. */
    public double ballDiameterInches = DEFAULT_BALL_DIAMETER_INCHES;
    /** Lens height above the floor. */
    public double cameraHeightInches = DEFAULT_CAMERA_HEIGHT_INCHES;
    /**
     * Downward pitch of the optical axis, degrees. 0 = level.
     * Measure with a phone inclinometer on the housing, or
     * {@link EasyOBJDCalibration#suggestedTiltDegrees}.
     */
    public double cameraTiltDegrees = DEFAULT_CAMERA_TILT_DEGREES;
    /**
     * Horizontal FOV at the stream width. Used when
     * {@link #focalLengthPixelsAt640} is 0.
     */
    public double horizontalFovDegrees = DEFAULT_HORIZONTAL_FOV_DEGREES;
    /**
     * 0 = derive from {@link #horizontalFovDegrees}. Set after tape
     * calibration via {@link EasyOBJDCalibration#focalLengthAt640}.
     */
    public double focalLengthPixelsAt640 = DEFAULT_FOCAL_LENGTH_PIXELS_AT_640;
    public double cameraForwardOfCenter = DEFAULT_CAMERA_FORWARD_OF_CENTER;
    public double cameraRightOfCenter = DEFAULT_CAMERA_RIGHT_OF_CENTER;
    public double cameraYawDegrees = DEFAULT_CAMERA_YAW_DEGREES;

    public int gridRows = DEFAULT_GRID_ROWS;
    public int gridCols = DEFAULT_GRID_COLS;
    /** Fraction of a cell that must be yellow to count as occupied. */
    public double whiteRatioThreshold = DEFAULT_WHITE_RATIO_THRESHOLD;
    /** How far (px at 640-wide) a circular ball may sit from the cluster centroid. */
    public double ballSearchRadiusPx = DEFAULT_BALL_SEARCH_RADIUS_PX;

    /**
     * Hole / far-object cutoff. Apparent size of this diameter at
     * {@link #maxRangeInches} is the smallest circle accepted.
     */
    public double minBallDiameterInches = DEFAULT_MIN_BALL_DIAMETER_INCHES;
    /** Ignore balls / clusters farther than this (default 5 feet). */
    public double maxRangeInches = DEFAULT_MAX_RANGE_INCHES;

    /**
     * 1.0 = process at the camera stream size. 0.5 at 640×480 is 320×240
     * (4× fewer pixels). Preview stays at stream resolution.
     * Prefer {@link #processWidth} when you want process size independent
     * of the preview.
     */
    public double processScale = DEFAULT_PROCESS_SCALE;
    /**
     * If &gt; 0, process at this width (height follows aspect) and ignore
     * {@link #processScale}. Recommended: 320 for Control Hub cycle time,
     * 640 when you need distant balls.
     */
    public int processWidth = 0;

    /**
     * Primary HSV in-range (OpenCV 8-bit: H 0–179, S/V 0–255).
     * Defaults are yellow game pieces; set via {@link Builder#hsv} for other seasons.
     */
    public Scalar hsvLower = new Scalar(DEFAULT_H_LOWER, DEFAULT_S_LOWER, DEFAULT_V_LOWER);
    public Scalar hsvUpper = new Scalar(DEFAULT_H_UPPER, DEFAULT_S_UPPER, DEFAULT_V_UPPER);
    /** @deprecated use {@link #hsvLower} */
    public Scalar yellowLower = hsvLower;
    /** @deprecated use {@link #hsvUpper} */
    public Scalar yellowUpper = hsvUpper;
    /**
     * Extra HSV ranges OR'd into the mask (other game-piece colors).
     */
    public final List<ColorRange> extraColorRanges = new ArrayList<ColorRange>();
    /**
     * When the frame is dark, relax S/V lower bounds slightly so arena
     * lighting changes do not punch holes in the mask.
     */
    public boolean adaptiveLighting = false;

    /**
     * 0 = size from process width (≈5 / 9 px at 640-wide). Odd values recommended.
     */
    public int openKernelSize = 0;
    public int closeKernelSize = 0;

    public double singleBallMinCircularity = 0.75;
    public double singleBallMinFill = 0.75;
    public double minCircularity = 0.55;
    public double minCircleMaskFill = 0.60;
    public double ballOverlapFrac = 0.55;
    /** Reject single-ball contours skinnier than this (max/min side). */
    public double maxAspectRatio = 1.65;
    /**
     * After localization, reject if apparent radius is below this fraction
     * of the expected size at that range (partial / hole rejection).
     */
    public double minExpectedSizeFrac = 0.40;
    public double maxExpectedSizeFrac = 2.50;
    /** Drop detections above the horizon (ceiling lights, audience). */
    public boolean verticalBandFilter = true;

    public double arcRmsPx = 3.5;
    public double arcCenterMergeFrac = 0.40;
    public double arcRadiusMergeFrac = 0.30;
    public int minArcVotes = 2;
    public int maxBallsPerBlob = 8;
    /** Cap sliding windows on large contours so arc-split stays cheap. */
    public int maxArcWindows = 20;

    /**
     * Optional HoughCircles pass used as extra candidates (default off).
     * Enable only after confirming cycle time on the Control Hub.
     */
    public boolean useHoughCircles = false;

    /**
     * Exponential smoothing on published cluster X/Y. 0 = raw frame-to-frame
     * (best for high-speed decisions). 0.25–0.40 damps jitter for driving.
     */
    public double smoothingAlpha = 0;

    public OverlayMode overlayMode = OverlayMode.FULL;

    public EasyOBJDConfig() {}

    public static Builder builder() {
        return new Builder();
    }

    public static EasyOBJDConfig defaults() {
        return new EasyOBJDConfig();
    }

    /** Snapshot of the current {@link EasyOBJDPipeline} static defaults. */
    public static EasyOBJDConfig fromPipelineStatics() {
        EasyOBJDConfig c = new EasyOBJDConfig();
        c.cameraHeightInches = EasyOBJDPipeline.CAMERA_HEIGHT_INCHES;
        c.cameraTiltDegrees = EasyOBJDPipeline.CAMERA_TILT_DEGREES;
        c.horizontalFovDegrees = EasyOBJDPipeline.HORIZONTAL_FOV_DEGREES;
        c.focalLengthPixelsAt640 = EasyOBJDPipeline.FOCAL_LENGTH_PIXELS_AT_640;
        c.cameraForwardOfCenter = EasyOBJDPipeline.CAMERA_FORWARD_OF_CENTER;
        c.cameraRightOfCenter = EasyOBJDPipeline.CAMERA_RIGHT_OF_CENTER;
        c.cameraYawDegrees = Math.toDegrees(EasyOBJDPipeline.CAMERA_YAW_RAD);
        c.gridRows = EasyOBJDPipeline.GRID_ROWS;
        c.gridCols = EasyOBJDPipeline.GRID_COLS;
        c.whiteRatioThreshold = EasyOBJDPipeline.WHITE_RATIO_THRESHOLD;
        c.ballSearchRadiusPx = EasyOBJDPipeline.BALL_SEARCH_RADIUS_PX;
        c.minBallDiameterInches = EasyOBJDPipeline.MIN_BALL_DIAMETER_INCHES;
        c.maxRangeInches = EasyOBJDPipeline.MAX_RANGE_INCHES;
        c.processScale = EasyOBJDPipeline.PROCESS_SCALE;
        c.hsvLower = EasyOBJDPipeline.yellowLower.clone();
        c.hsvUpper = EasyOBJDPipeline.yellowUpper.clone();
        c.yellowLower = c.hsvLower;
        c.yellowUpper = c.hsvUpper;
        return c;
    }

    public double cameraYawRad() {
        return Math.toRadians(cameraYawDegrees);
    }

    /**
     * delta &gt; 0 widens the current HSV band, delta &lt; 0 tightens.
     * Color-agnostic (H 0–179). Safe from TeleOp.
     */
    public void nudgeHsvRange(int delta) {
        nudgeHsv(hsvLower, hsvUpper, delta);
        yellowLower = hsvLower;
        yellowUpper = hsvUpper;
    }

    /** Widen / tighten any HSV pair. Hue is not clamped to yellow. */
    public static void nudgeHsv(Scalar lower, Scalar upper, int delta) {
        if (delta == 0 || lower == null || upper == null) {
            return;
        }
        lower.val[0] = clamp(lower.val[0] - delta, 0, 179);
        upper.val[0] = clamp(upper.val[0] + delta, 0, 179);
        lower.val[1] = clamp(lower.val[1] - 5 * delta, 0, 255);
        lower.val[2] = clamp(lower.val[2] - 5 * delta, 0, 255);
        if (lower.val[0] > upper.val[0]) {
            double tmp = lower.val[0];
            lower.val[0] = upper.val[0];
            upper.val[0] = tmp;
        }
    }

    public EasyOBJDConfig copy() {
        EasyOBJDConfig c = new EasyOBJDConfig();
        c.ballDiameterInches = ballDiameterInches;
        c.cameraHeightInches = cameraHeightInches;
        c.cameraTiltDegrees = cameraTiltDegrees;
        c.horizontalFovDegrees = horizontalFovDegrees;
        c.focalLengthPixelsAt640 = focalLengthPixelsAt640;
        c.cameraForwardOfCenter = cameraForwardOfCenter;
        c.cameraRightOfCenter = cameraRightOfCenter;
        c.cameraYawDegrees = cameraYawDegrees;
        c.gridRows = gridRows;
        c.gridCols = gridCols;
        c.whiteRatioThreshold = whiteRatioThreshold;
        c.ballSearchRadiusPx = ballSearchRadiusPx;
        c.minBallDiameterInches = minBallDiameterInches;
        c.maxRangeInches = maxRangeInches;
        c.processScale = processScale;
        c.processWidth = processWidth;
        c.hsvLower = (hsvLower != null ? hsvLower : yellowLower).clone();
        c.hsvUpper = (hsvUpper != null ? hsvUpper : yellowUpper).clone();
        c.yellowLower = c.hsvLower;
        c.yellowUpper = c.hsvUpper;
        c.extraColorRanges.clear();
        for (ColorRange range : extraColorRanges) {
            c.extraColorRanges.add(range.copy());
        }
        c.adaptiveLighting = adaptiveLighting;
        c.openKernelSize = openKernelSize;
        c.closeKernelSize = closeKernelSize;
        c.singleBallMinCircularity = singleBallMinCircularity;
        c.singleBallMinFill = singleBallMinFill;
        c.minCircularity = minCircularity;
        c.minCircleMaskFill = minCircleMaskFill;
        c.ballOverlapFrac = ballOverlapFrac;
        c.maxAspectRatio = maxAspectRatio;
        c.minExpectedSizeFrac = minExpectedSizeFrac;
        c.maxExpectedSizeFrac = maxExpectedSizeFrac;
        c.verticalBandFilter = verticalBandFilter;
        c.arcRmsPx = arcRmsPx;
        c.arcCenterMergeFrac = arcCenterMergeFrac;
        c.arcRadiusMergeFrac = arcRadiusMergeFrac;
        c.minArcVotes = minArcVotes;
        c.maxBallsPerBlob = maxBallsPerBlob;
        c.maxArcWindows = maxArcWindows;
        c.useHoughCircles = useHoughCircles;
        c.smoothingAlpha = smoothingAlpha;
        c.overlayMode = overlayMode;
        return c;
    }

    static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** One additional HSV in-range, OR'd into the primary yellow mask. */
    public static final class ColorRange {
        public Scalar lower;
        public Scalar upper;

        public ColorRange(Scalar lower, Scalar upper) {
            this.lower = lower;
            this.upper = upper;
        }

        public ColorRange copy() {
            return new ColorRange(lower.clone(), upper.clone());
        }
    }

    public static final class Builder {
        private final EasyOBJDConfig c = new EasyOBJDConfig();

        public Builder camera(double heightInches, double tiltDegrees, double horizontalFovDegrees) {
            c.cameraHeightInches = heightInches;
            c.cameraTiltDegrees = tiltDegrees;
            c.horizontalFovDegrees = horizontalFovDegrees;
            return this;
        }

        public Builder cameraOffsets(double forwardInches, double rightInches, double yawDegrees) {
            c.cameraForwardOfCenter = forwardInches;
            c.cameraRightOfCenter = rightInches;
            c.cameraYawDegrees = yawDegrees;
            return this;
        }

        public Builder focalLengthPixelsAt640(double focal) {
            c.focalLengthPixelsAt640 = focal;
            return this;
        }

        public Builder processScale(double scale) {
            c.processScale = scale;
            return this;
        }

        public Builder processWidth(int width) {
            c.processWidth = width;
            return this;
        }

        /**
         * Primary detection color. OpenCV 8-bit HSV: H 0–179, S/V 0–255.
         * Yellow defaults are {@code 21, 95, 85} … {@code 35, 255, 255}.
         */
        public Builder hsv(double hLo, double sLo, double vLo, double hHi, double sHi, double vHi) {
            return hsv(new Scalar(hLo, sLo, vLo), new Scalar(hHi, sHi, vHi));
        }

        public Builder hsv(Scalar lower, Scalar upper) {
            c.hsvLower = lower.clone();
            c.hsvUpper = upper.clone();
            c.yellowLower = c.hsvLower;
            c.yellowUpper = c.hsvUpper;
            return this;
        }

        /**
         * Official game-piece diameter used for size-based range and the
         * floor-plane height (ball center = diameter / 2).
         */
        public Builder ballDiameterInches(double inches) {
            c.ballDiameterInches = inches;
            return this;
        }

        /**
         * Smallest apparent object accepted (hole / far-object cutoff).
         * Defaults to 3.0 in — slightly larger than a 2.8 in pollen.
         */
        public Builder minBallDiameterInches(double inches) {
            c.minBallDiameterInches = inches;
            return this;
        }

        /**
         * Sets official diameter and a slightly larger hole-cutoff
         * ({@code max(diameter, diameter + 0.2)}).
         */
        public Builder ballSize(double diameterInches) {
            c.ballDiameterInches = diameterInches;
            c.minBallDiameterInches = Math.max(diameterInches, diameterInches + 0.2);
            return this;
        }

        public Builder extraColorRange(Scalar lower, Scalar upper) {
            c.extraColorRanges.add(new ColorRange(lower, upper));
            return this;
        }

        public Builder grid(int rows, int cols) {
            c.gridRows = rows;
            c.gridCols = cols;
            return this;
        }

        public Builder maxRangeInches(double inches) {
            c.maxRangeInches = inches;
            return this;
        }

        public Builder adaptiveLighting(boolean enabled) {
            c.adaptiveLighting = enabled;
            return this;
        }

        public Builder smoothingAlpha(double alpha) {
            c.smoothingAlpha = alpha;
            return this;
        }

        public Builder overlayMode(OverlayMode mode) {
            c.overlayMode = mode;
            return this;
        }

        public Builder useHoughCircles(boolean enabled) {
            c.useHoughCircles = enabled;
            return this;
        }

        public EasyOBJDConfig build() {
            return c.copy();
        }
    }
}
