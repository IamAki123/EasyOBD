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
 * Factory for the EasyOBJD game-piece pipeline, plus the small public enums
 * teams pass into config and intake.
 *
 * <pre>
 * EasyOBJDPipeline pipeline = EasyOBJD.createPipeline();
 * webcam.setPipeline(pipeline);
 * </pre>
 *
 * Optional: pass {@link EasyOBJDConfig} (or a TeamCode {@code EasyOBJDUserConfig})
 * when you want saved HSV / camera inches instead of library defaults.
 */
public final class EasyOBJD {
    private EasyOBJD() {}

    public static EasyOBJDPipeline createPipeline() {
        return new EasyOBJDPipeline();
    }

    public static EasyOBJDPipeline createPipeline(EasyOBJDConfig config) {
        return new EasyOBJDPipeline(config);
    }

    /** What {@link EasyOBJDPipeline} draws on the preview Mat. */
    public enum OverlayMode {
        /** Yellow mask, grid, balls, cluster markers, and range labels. */
        FULL,
        /** Binary yellow mask only. */
        MASK,
        /** Mask plus occupied-cell tint and grid lines. */
        GRID,
        /** Mask plus fitted ball circles. */
        BALLS,
        /** Mask, balls, cluster markers, and distance text. */
        DISTANCES
    }

    /**
     * How a cluster's camera-relative X/Y was computed.
     *
     * <p>{@link #FLOOR_PLANE} is preferred whenever the pixel ray intersects the
     * ball-center plane (camera height + downward tilt) inside
     * {@link EasyOBJDConfig#maxRangeInches}. {@link #SIZE_BASED} is the fallback
     * when the ray misses that plane (level camera, ball above the horizon, or
     * extreme tilt) and an apparent radius is available.
     */
    public enum LocalizationMethod {
        /** Ray–floor intersection using camera height and pitch. */
        FLOOR_PLANE,
        /** Pinhole range from known ball diameter vs apparent pixel radius. */
        SIZE_BASED,
        /** No valid localization (should not appear on published clusters). */
        NONE
    }

    /** Team-specific pick for {@link EasyOBJDPipeline#getBestClusterForIntake}. */
    public enum IntakeHeuristic {
        /** Smallest planar range from the lens (closest game piece). */
        CLOSEST,
        /** Smallest camera X (leftmost in the image / to the robot's left). */
        LEFTMOST,
        /** Highest combined circularity / fill / localization confidence. */
        HIGHEST_CONFIDENCE
    }
}
