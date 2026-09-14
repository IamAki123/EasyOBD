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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * One yellow group, numbered left-to-right.
 * {@link #x} = inches right of the lens, {@link #y} = inches forward of the lens.
 */
public final class ClusterInfo {
    public final int id;
    public final double x;
    public final double y;
    public final Point centerPx;
    public final List<Integer> cells;
    /** Planar range from the lens, {@code hypot(x, y)}. */
    public final double rangeInches;
    public final int ballCount;
    public final double circularity;
    public final double confidence;
    public final double pixelRadius;
    public final LocalizationMethod localization;
    public final List<Ball> balls;
    /**
     * RMS ball-center spread in the image, divided by mean radius.
     * 0 if the cluster has fewer than two associated balls.
     */
    public final double tightness;
    /** Field X after {@link EasyOBJDPipeline#setRobotPose}; otherwise NaN. */
    public final double fieldX;
    /** Field Y after {@link EasyOBJDPipeline#setRobotPose}; otherwise NaN. */
    public final double fieldY;

    public ClusterInfo(int id, double x, double y, Point centerPx, List<Integer> cells) {
        this(id, x, y, centerPx, cells, Math.hypot(x, y), 0, 0, 0, 0,
                LocalizationMethod.NONE, Collections.<Ball>emptyList(), 0,
                Double.NaN, Double.NaN);
    }

    public ClusterInfo(int id, double x, double y, Point centerPx, List<Integer> cells,
                       double rangeInches, int ballCount, double circularity, double confidence,
                       double pixelRadius, LocalizationMethod localization, List<Ball> balls,
                       double tightness, double fieldX, double fieldY) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.centerPx = centerPx == null ? new Point() : new Point(centerPx.x, centerPx.y);
        this.cells = cells == null
                ? Collections.<Integer>emptyList()
                : Collections.unmodifiableList(new ArrayList<Integer>(cells));
        this.rangeInches = rangeInches;
        this.ballCount = ballCount;
        this.circularity = circularity;
        this.confidence = confidence;
        this.pixelRadius = pixelRadius;
        this.localization = localization == null ? LocalizationMethod.NONE : localization;
        this.balls = balls == null
                ? Collections.<Ball>emptyList()
                : Collections.unmodifiableList(new ArrayList<Ball>(balls));
        this.tightness = tightness;
        this.fieldX = fieldX;
        this.fieldY = fieldY;
    }

    ClusterInfo withIdAndPose(int newId, double newX, double newY, double newFieldX, double newFieldY) {
        return new ClusterInfo(newId, newX, newY, centerPx, cells,
                Math.hypot(newX, newY), ballCount, circularity, confidence, pixelRadius,
                localization, balls, tightness, newFieldX, newFieldY);
    }
}
