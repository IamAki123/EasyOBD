/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobd;

import org.opencv.core.Point;

/**
 * One circular game piece. Camera-relative inches: {@link #x} right,
 * {@link #z} forward (always {@link LocalizationMethod#SIZE_BASED}).
 */
public final class Ball {
    public final Point center;
    public final double radiusPx;
    public final double area;
    public final double circularity;
    public final double x;
    public final double y;
    public final double z;
    /** {@link #z} when in range; otherwise NaN. */
    public final double rangeInches;
    public final double confidence;
    public final LocalizationMethod localization;

    public Ball(Point center, double radiusPx, double area, double circularity,
                double x, double y, double z, double confidence) {
        this.center = center == null ? new Point() : new Point(center.x, center.y);
        this.radiusPx = radiusPx;
        this.area = area;
        this.circularity = circularity;
        this.x = x;
        this.y = y;
        this.z = z;
        this.rangeInches = z;
        this.confidence = confidence;
        this.localization = LocalizationMethod.SIZE_BASED;
    }
}
