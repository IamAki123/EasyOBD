/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobd;

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
