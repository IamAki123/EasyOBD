package org.firstinspires.ftc.easyobd;

import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** One yellow group, numbered left-to-right. X = right of lens, Y = forward of lens, inches. */
public final class ClusterInfo {
    public final int id;
    public final double x;
    public final double y;
    public final Point centerPx;
    public final List<Integer> cells;

    public ClusterInfo(int id, double x, double y, Point centerPx, List<Integer> cells) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.centerPx = centerPx;
        this.cells = cells == null
                ? Collections.emptyList()
                : Collections.unmodifiableList(new ArrayList<>(cells));
    }
}
