/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobd;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

/**
 * 4-connected occupied-cell grouping and grid sizing. Pure Java for tests.
 */
public final class GridClustering {
    private GridClustering() {}

    /**
     * Clamp a requested grid dimension so each cell is at least
     * {@code minCellPx} on a {@code sizePx}-wide (or tall) frame.
     * Extreme aspect ratios and tiny process frames are guarded here.
     */
    public static int resolveGridDim(int requested, int sizePx, int minCellPx) {
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
    public static List<List<Integer>> groupTouching(boolean[] occupied, int rows, int cols) {
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

    public static boolean cellContainsPoint(List<Integer> cells, int rows, int cols,
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
    public static double tightness(double[] xs, double[] ys, double scale) {
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
