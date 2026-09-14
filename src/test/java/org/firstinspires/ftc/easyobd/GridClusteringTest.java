/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobd;

import org.junit.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class GridClusteringTest {

    @Test
    public void twoSeparatedBlobs() {
        boolean[] occupied = {
                true, true, false,
                false, false, false,
                false, true, true
        };
        List<List<Integer>> groups = GridClustering.groupTouching(occupied, 3, 3);
        assertEquals(2, groups.size());
        assertEquals(2, groups.get(0).size());
        assertEquals(2, groups.get(1).size());
    }

    @Test
    public void touchingCellsMerge() {
        boolean[] occupied = {
                true, true, false,
                true, false, false,
                false, false, false
        };
        List<List<Integer>> groups = GridClustering.groupTouching(occupied, 3, 3);
        assertEquals(1, groups.size());
        assertEquals(3, groups.get(0).size());
    }

    @Test
    public void emptyGrid() {
        boolean[] occupied = new boolean[9];
        assertTrue(GridClustering.groupTouching(occupied, 3, 3).isEmpty());
    }

    @Test
    public void resolveGridDimGuardsTinyFrames() {
        assertEquals(2, GridClustering.resolveGridDim(12, 10, 4));
        assertEquals(12, GridClustering.resolveGridDim(12, 640, 4));
        assertEquals(1, GridClustering.resolveGridDim(12, 0, 4));
    }

    @Test
    public void cellContainsPoint() {
        List<Integer> cells = Arrays.asList(0, 1);
        assertTrue(GridClustering.cellContainsPoint(cells, 2, 2, 10, 10, 100, 100));
        assertFalse(GridClustering.cellContainsPoint(cells, 2, 2, 80, 80, 100, 100));
    }

    @Test
    public void tightnessZeroForSinglePoint() {
        assertEquals(0, GridClustering.tightness(new double[] {1}, new double[] {2}, 5), 0);
    }

    @Test
    public void tightnessScalesWithSpread() {
        double tight = GridClustering.tightness(new double[] {0, 2}, new double[] {0, 0}, 10);
        double loose = GridClustering.tightness(new double[] {0, 20}, new double[] {0, 0}, 10);
        assertTrue(loose > tight);
    }
}
