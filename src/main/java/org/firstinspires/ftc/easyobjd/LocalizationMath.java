/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobjd;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

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

/**
 * 4-connected occupied-cell grouping and grid sizing. Pure Java for tests.
 * Lives in this file so the AAR does not ship a second tiny public class.
 */
final class GridClustering {
    private GridClustering() {}

    /**
     * Clamp a requested grid dimension so each cell is at least
     * {@code minCellPx} on a {@code sizePx}-wide (or tall) frame.
     * Extreme aspect ratios and tiny process frames are guarded here.
     */
    static int resolveGridDim(int requested, int sizePx, int minCellPx) {
        if (requested < 1) {
            return 1;
        }
        if (sizePx < 1) {
            return 1;
        }
        int minPx = Math.max(2, minCellPx);
        int maxDim = Math.max(1, sizePx / minPx);
        return Math.max(1, Math.min(requested, maxDim));
    }

    /** 4-connected components of {@code occupied} in row-major order. */
    static List<List<Integer>> groupTouching(boolean[] occupied, int rows, int cols) {
        List<List<Integer>> states = new ArrayList<>();
        if (occupied == null || rows < 1 || cols < 1 || occupied.length < rows * cols) {
            return states;
        }
        boolean[] visited = new boolean[rows * cols];
        int[] dr = {-1, 1, 0, 0};
        int[] dc = {0, 0, -1, 1};

        for (int start = 0; start < rows * cols; start++) {
            if (!occupied[start] || visited[start]) {
                continue;
            }
            List<Integer> state = new ArrayList<>();
            Queue<Integer> queue = new ArrayDeque<>();
            queue.add(start);
            visited[start] = true;
            while (!queue.isEmpty()) {
                int cur = queue.poll();
                state.add(cur);
                int r = cur / cols;
                int c = cur % cols;
                for (int i = 0; i < 4; i++) {
                    int nr = r + dr[i];
                    int nc = c + dc[i];
                    if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) {
                        continue;
                    }
                    int next = nr * cols + nc;
                    if (occupied[next] && !visited[next]) {
                        visited[next] = true;
                        queue.add(next);
                    }
                }
            }
            states.add(state);
        }
        return states;
    }

    static boolean cellContainsPoint(List<Integer> cells, int rows, int cols,
                                    double px, double py, int width, int height) {
        if (cells == null || cells.isEmpty() || width <= 0 || height <= 0) {
            return false;
        }
        int col = (int) Math.min(cols - 1, Math.max(0, px * cols / width));
        int row = (int) Math.min(rows - 1, Math.max(0, py * rows / height));
        return cells.contains(row * cols + col);
    }

    /**
     * RMS distance of points from their centroid, divided by {@code scale}
     * (typically mean ball radius). 0 if fewer than two points.
     */
    static double tightness(double[] xs, double[] ys, double scale) {
        if (xs == null || ys == null || xs.length < 2 || xs.length != ys.length) {
            return 0;
        }
        double mx = 0;
        double my = 0;
        for (int i = 0; i < xs.length; i++) {
            mx += xs[i];
            my += ys[i];
        }
        mx /= xs.length;
        my /= ys.length;
        double acc = 0;
        for (int i = 0; i < xs.length; i++) {
            acc += (xs[i] - mx) * (xs[i] - mx) + (ys[i] - my) * (ys[i] - my);
        }
        double rms = Math.sqrt(acc / xs.length);
        return rms / Math.max(scale, 1e-6);
    }
}

