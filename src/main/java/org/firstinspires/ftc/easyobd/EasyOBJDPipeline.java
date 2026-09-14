/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobd;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Easy Object Detection pipeline: yellow HSV mask, 12×12 grid clustering,
 * then circular / arc-split balls near each cluster.
 *
 * <h2>Coordinate frames</h2>
 * Camera-relative inches (no odometry needed): {@link #getClusterX()} is
 * right of the lens (+ right), {@link #getClusterY()} is forward of the lens.
 * Field-frame {@code FieldX}/{@code FieldY} and {@link ClusterInfo#fieldX}
 * only update after {@link #setRobotPose}.
 *
 * <h2>Localization</h2>
 * {@link LocalizationMethod#FLOOR_PLANE} is used when the pixel ray hits the
 * ball-center plane inside {@link EasyOBJDConfig#maxRangeInches}.
 * {@link LocalizationMethod#SIZE_BASED} is the fallback from apparent radius.
 * See {@link ClusterInfo#localization}.
 *
 * <h2>Thread safety</h2>
 * {@link #processFrame} runs on the EasyOpenCV camera thread. Getters read
 * an immutable snapshot published at the end of each frame (copy-on-write).
 * Do not mutate returned lists or {@link ClusterInfo}/{@link Ball} contents.
 *
 * <h2>Resolution</h2>
 * Stream at 640×480 for preview; set {@link EasyOBJDConfig#processWidth} to
 * 320 (or {@code processScale = 0.5}) on a Control Hub if cycle time is high.
 * Processing is independent of the preview size when {@code processWidth} is set.
 */
public class EasyOBJDPipeline extends OpenCvPipeline {

    /**
     * One field-axis reading. {@link #bestClusterCenterpoint()} is NaN until
     * a cluster is locked and {@link #setRobotPose} has been called.
     */
    public static final class ClusterAxis {
        private volatile double value = Double.NaN;

        public double bestClusterCenterpoint() {
            return value;
        }

        void set(double next) {
            value = next;
        }
    }

    public final ClusterAxis FieldX = new ClusterAxis();
    public final ClusterAxis FieldY = new ClusterAxis();

    // ---- Defaults (copied into a new pipeline; prefer EasyOBJDConfig). ----
    public static int GRID_ROWS = EasyOBJDConfig.DEFAULT_GRID_ROWS;
    public static int GRID_COLS = EasyOBJDConfig.DEFAULT_GRID_COLS;
    public static double WHITE_RATIO_THRESHOLD = EasyOBJDConfig.DEFAULT_WHITE_RATIO_THRESHOLD;
    public static double BALL_SEARCH_RADIUS_PX = EasyOBJDConfig.DEFAULT_BALL_SEARCH_RADIUS_PX;
    public static Scalar yellowLower = new Scalar(
            EasyOBJDConfig.DEFAULT_H_LOWER, EasyOBJDConfig.DEFAULT_S_LOWER, EasyOBJDConfig.DEFAULT_V_LOWER);
    public static Scalar yellowUpper = new Scalar(
            EasyOBJDConfig.DEFAULT_H_UPPER, EasyOBJDConfig.DEFAULT_S_UPPER, EasyOBJDConfig.DEFAULT_V_UPPER);
    public static double MIN_BALL_DIAMETER_INCHES = EasyOBJDConfig.DEFAULT_MIN_BALL_DIAMETER_INCHES;
    public static double MAX_RANGE_INCHES = EasyOBJDConfig.DEFAULT_MAX_RANGE_INCHES;
    public static double HORIZONTAL_FOV_DEGREES = EasyOBJDConfig.DEFAULT_HORIZONTAL_FOV_DEGREES;
    public static double FOCAL_LENGTH_PIXELS_AT_640 = EasyOBJDConfig.DEFAULT_FOCAL_LENGTH_PIXELS_AT_640;
    public static double CAMERA_HEIGHT_INCHES = EasyOBJDConfig.DEFAULT_CAMERA_HEIGHT_INCHES;
    public static double CAMERA_TILT_DEGREES = EasyOBJDConfig.DEFAULT_CAMERA_TILT_DEGREES;
    public static double CAMERA_FORWARD_OF_CENTER = EasyOBJDConfig.DEFAULT_CAMERA_FORWARD_OF_CENTER;
    public static double CAMERA_RIGHT_OF_CENTER = EasyOBJDConfig.DEFAULT_CAMERA_RIGHT_OF_CENTER;
    public static double CAMERA_YAW_RAD = Math.toRadians(EasyOBJDConfig.DEFAULT_CAMERA_YAW_DEGREES);
    public static double PROCESS_SCALE = EasyOBJDConfig.DEFAULT_PROCESS_SCALE;

    private final EasyOBJDConfig cfg;

    private final Mat process = new Mat();
    private final Mat rgbMat = new Mat();
    private final Mat blurred = new Mat();
    private final Mat hsvMat = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat extraMask = new Mat();
    private final Mat integral = new Mat();
    private final Mat clusterMask = new Mat();
    private final Mat preview = new Mat();
    private final Mat overlay = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat circleDisk = new Mat();
    private final Mat circleAnd = new Mat();
    private final Mat houghBuf = new Mat();
    private final MatOfPoint2f contour2fScratch = new MatOfPoint2f();
    private final List<MatOfPoint> contourBuf = new ArrayList<MatOfPoint>();
    private final float[] circleRadiusBuf = new float[1];
    private final Scalar hsvLowerScratch = new Scalar(0, 0, 0);
    private Mat openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
    private Mat closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(9, 9));
    private int lastOpenSize = 5;
    private int lastCloseSize = 9;

    private int[] integralData = new int[0];
    private int integralCols = 0;

    private volatile double robotX = 0;
    private volatile double robotY = 0;
    private volatile double robotHeading = 0;
    private volatile boolean poseSet = false;

    private volatile long frameCount = 0;
    private volatile long lastProcessTimeNs = 0;
    private volatile String lastError = null;

    private volatile int occupiedCount = 0;
    private volatile int stateCount = 0;
    private volatile int bestStateSize = 0;
    private volatile boolean clusterValid = false;
    private volatile double clusterPixelX = Double.NaN;
    private volatile double clusterPixelY = Double.NaN;
    private volatile double clusterCamX = Double.NaN;
    private volatile double clusterCamZ = Double.NaN;
    private volatile double clusterRobotX = Double.NaN;
    private volatile double clusterRobotY = Double.NaN;
    private volatile double clusterSizeRangeIn = Double.NaN;
    private volatile double clusterRadiusPx = Double.NaN;
    private volatile boolean clusterUsedFloorPlane = false;
    private volatile int ballsNearCluster = 0;

    private final AtomicReference<List<Ball>> ballSnapshot =
            new AtomicReference<List<Ball>>(Collections.<Ball>emptyList());
    private final AtomicReference<List<ClusterInfo>> clusterSnapshot =
            new AtomicReference<List<ClusterInfo>>(Collections.<ClusterInfo>emptyList());
    private final List<double[]> prevClusterXy = new ArrayList<double[]>();

    public EasyOBJDPipeline() {
        this(EasyOBJDConfig.fromPipelineStatics());
    }

    public EasyOBJDPipeline(EasyOBJDConfig config) {
        this.cfg = config == null ? EasyOBJDConfig.defaults() : config;
    }

    /** Live-tunable config for this pipeline (not shared with other instances). */
    public EasyOBJDConfig getConfig() {
        return cfg;
    }

    public void setConfig(EasyOBJDConfig replacement) {
        if (replacement == null) {
            return;
        }
        cfg.ballDiameterInches = replacement.ballDiameterInches;
        cfg.cameraHeightInches = replacement.cameraHeightInches;
        cfg.cameraTiltDegrees = replacement.cameraTiltDegrees;
        cfg.horizontalFovDegrees = replacement.horizontalFovDegrees;
        cfg.focalLengthPixelsAt640 = replacement.focalLengthPixelsAt640;
        cfg.cameraForwardOfCenter = replacement.cameraForwardOfCenter;
        cfg.cameraRightOfCenter = replacement.cameraRightOfCenter;
        cfg.cameraYawDegrees = replacement.cameraYawDegrees;
        cfg.gridRows = replacement.gridRows;
        cfg.gridCols = replacement.gridCols;
        cfg.whiteRatioThreshold = replacement.whiteRatioThreshold;
        cfg.ballSearchRadiusPx = replacement.ballSearchRadiusPx;
        cfg.minBallDiameterInches = replacement.minBallDiameterInches;
        cfg.maxRangeInches = replacement.maxRangeInches;
        cfg.processScale = replacement.processScale;
        cfg.processWidth = replacement.processWidth;
        Scalar srcLower = replacement.hsvLower != null ? replacement.hsvLower : replacement.yellowLower;
        Scalar srcUpper = replacement.hsvUpper != null ? replacement.hsvUpper : replacement.yellowUpper;
        cfg.hsvLower = srcLower.clone();
        cfg.hsvUpper = srcUpper.clone();
        cfg.yellowLower = cfg.hsvLower;
        cfg.yellowUpper = cfg.hsvUpper;
        cfg.extraColorRanges.clear();
        for (EasyOBJDConfig.ColorRange range : replacement.extraColorRanges) {
            cfg.extraColorRanges.add(range.copy());
        }
        cfg.adaptiveLighting = replacement.adaptiveLighting;
        cfg.openKernelSize = replacement.openKernelSize;
        cfg.closeKernelSize = replacement.closeKernelSize;
        cfg.singleBallMinCircularity = replacement.singleBallMinCircularity;
        cfg.singleBallMinFill = replacement.singleBallMinFill;
        cfg.minCircularity = replacement.minCircularity;
        cfg.minCircleMaskFill = replacement.minCircleMaskFill;
        cfg.ballOverlapFrac = replacement.ballOverlapFrac;
        cfg.maxAspectRatio = replacement.maxAspectRatio;
        cfg.minExpectedSizeFrac = replacement.minExpectedSizeFrac;
        cfg.maxExpectedSizeFrac = replacement.maxExpectedSizeFrac;
        cfg.verticalBandFilter = replacement.verticalBandFilter;
        cfg.arcRmsPx = replacement.arcRmsPx;
        cfg.arcCenterMergeFrac = replacement.arcCenterMergeFrac;
        cfg.arcRadiusMergeFrac = replacement.arcRadiusMergeFrac;
        cfg.minArcVotes = replacement.minArcVotes;
        cfg.maxBallsPerBlob = replacement.maxBallsPerBlob;
        cfg.maxArcWindows = replacement.maxArcWindows;
        cfg.useHoughCircles = replacement.useHoughCircles;
        cfg.smoothingAlpha = replacement.smoothingAlpha;
        cfg.overlayMode = replacement.overlayMode;
    }

    /** Instance HSV nudge. Prefer this over the static {@link #nudgeHsvRange(int)}. */
    public void adjustHsvRange(int delta) {
        cfg.nudgeHsvRange(delta);
    }

    /**
     * Mutates the process-wide default HSV limits (copied only by new
     * no-arg pipelines). Prefer {@link #adjustHsvRange(int)}.
     */
    public static void nudgeHsvRange(int delta) {
        if (delta == 0) {
            return;
        }
        EasyOBJDConfig.nudgeHsv(yellowLower, yellowUpper, delta);
    }

    public OverlayMode cycleOverlayMode() {
        OverlayMode[] values = OverlayMode.values();
        cfg.overlayMode = values[(cfg.overlayMode.ordinal() + 1) % values.length];
        return cfg.overlayMode;
    }

    /**
     * Push the current Pedro / odometry pose so pixel → field conversion
     * stays current. Units: inches and heading in radians, field frame.
     */
    public void setRobotPose(double x, double y, double headingRad) {
        robotX = x;
        robotY = y;
        robotHeading = headingRad;
        poseSet = true;
    }

    @Override
    public Mat processFrame(Mat input) {
        long start = System.nanoTime();
        try {
            Mat src = toProcessSize(input);
            int pw = src.cols();
            int ph = src.rows();
            if (pw < 8 || ph < 8) {
                throw new IllegalArgumentException("Frame too small: " + pw + "x" + ph);
            }

            Imgproc.GaussianBlur(src, blurred, new Size(5, 5), 0);
            toHsv(blurred, hsvMat);
            applyColorMask(hsvMat, yellowMask);
            ensureKernels(pw);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, openKernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, closeKernel);
            fillExternalHoles(yellowMask);

            buildIntegral(yellowMask);

            List<Ball> found = findBalls(yellowMask, pw, ph);
            ClusterResult grid = findClusters(yellowMask);
            List<ClusterInfo> clusters = publishAllClusters(grid, found, pw, ph);

            ballSnapshot.set(Collections.unmodifiableList(found));
            clusterSnapshot.set(Collections.unmodifiableList(clusters));

            drawPreview(input, grid, found, clusters);
            lastError = null;
        } catch (Exception e) {
            lastError = e.getClass().getSimpleName() + ": " + e.getMessage();
            invalidateCluster();
            if (!yellowMask.empty()) {
                Imgproc.cvtColor(yellowMask, preview, Imgproc.COLOR_GRAY2BGR);
            }
        }

        frameCount++;
        lastProcessTimeNs = System.nanoTime() - start;
        return preview.empty() ? input : preview;
    }

    // ---- HSV & morphology ----

    private Mat toProcessSize(Mat input) {
        if (cfg.processWidth > 0 && input.cols() != cfg.processWidth) {
            double scale = cfg.processWidth / (double) input.cols();
            Imgproc.resize(input, process, new Size(), scale, scale, Imgproc.INTER_AREA);
            return process;
        }
        if (cfg.processScale >= 0.999) {
            return input;
        }
        Imgproc.resize(input, process, new Size(), cfg.processScale, cfg.processScale, Imgproc.INTER_AREA);
        return process;
    }

    private void toHsv(Mat src, Mat dst) {
        if (src.channels() == 4) {
            Imgproc.cvtColor(src, rgbMat, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(rgbMat, dst, Imgproc.COLOR_RGB2HSV);
        } else {
            Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2HSV);
        }
    }

    private void applyColorMask(Mat hsv, Mat dst) {
        Scalar lower = cfg.hsvLower != null ? cfg.hsvLower : cfg.yellowLower;
        Scalar upper = cfg.hsvUpper != null ? cfg.hsvUpper : cfg.yellowUpper;
        if (cfg.adaptiveLighting) {
            Scalar mean = Core.mean(hsv);
            if (mean.val[2] < 80) {
                hsvLowerScratch.val[0] = lower.val[0];
                hsvLowerScratch.val[1] = Math.max(20, lower.val[1] - 20);
                hsvLowerScratch.val[2] = Math.max(20, lower.val[2] - 25);
                lower = hsvLowerScratch;
            }
        }
        Core.inRange(hsv, lower, upper, dst);
        for (EasyOBJDConfig.ColorRange range : cfg.extraColorRanges) {
            Core.inRange(hsv, range.lower, range.upper, extraMask);
            Core.bitwise_or(dst, extraMask, dst);
        }
    }

    private void ensureKernels(int width) {
        int open = cfg.openKernelSize > 0
                ? oddAtLeast(cfg.openKernelSize, 3)
                : oddAtLeast((int) Math.round(5.0 * width / LocalizationMath.CALIBRATION_WIDTH), 3);
        int close = cfg.closeKernelSize > 0
                ? oddAtLeast(cfg.closeKernelSize, 3)
                : oddAtLeast((int) Math.round(9.0 * width / LocalizationMath.CALIBRATION_WIDTH), open + 2);
        if (open == lastOpenSize && close == lastCloseSize) {
            return;
        }
        openKernel.release();
        closeKernel.release();
        openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(open, open));
        closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(close, close));
        lastOpenSize = open;
        lastCloseSize = close;
    }

    private static int oddAtLeast(int value, int min) {
        int v = Math.max(min, value);
        if ((v & 1) == 0) {
            v++;
        }
        return v;
    }

    private void fillExternalHoles(Mat mask) {
        releaseContours(contourBuf);
        Imgproc.findContours(mask, contourBuf, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(mask, contourBuf, -1, new Scalar(255), -1);
        releaseContours(contourBuf);
    }

    private static void releaseContours(List<MatOfPoint> contours) {
        for (MatOfPoint contour : contours) {
            contour.release();
        }
        contours.clear();
    }

    // ---- Grid clustering ----

    private void buildIntegral(Mat mask) {
        Imgproc.integral(mask, integral);
        integralCols = integral.cols();
        int n = integral.rows() * integral.cols();
        if (integralData.length != n) {
            integralData = new int[n];
        }
        integral.get(0, 0, integralData);
    }

    private double regionWhiteRatio(Rect roi) {
        int x0 = roi.x;
        int y0 = roi.y;
        int x1 = roi.x + roi.width;
        int y1 = roi.y + roi.height;
        double sum = integralAt(x1, y1) - integralAt(x0, y1) - integralAt(x1, y0) + integralAt(x0, y0);
        return sum / (255.0 * roi.width * roi.height);
    }

    private int integralAt(int x, int y) {
        return integralData[y * integralCols + x];
    }

    private ClusterResult findClusters(Mat mask) {
        int width = mask.cols();
        int height = mask.rows();
        int rows = GridClustering.resolveGridDim(cfg.gridRows, height, 4);
        int cols = GridClustering.resolveGridDim(cfg.gridCols, width, 4);
        int cellCount = rows * cols;

        boolean[] occupied = new boolean[cellCount];
        double[] whiteRatio = new double[cellCount];
        int occupiedFound = 0;

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                Rect roi = cellRect(c, r, width, height, cols, rows);
                int idx = r * cols + c;
                double ratio = regionWhiteRatio(roi);
                whiteRatio[idx] = ratio;
                occupied[idx] = ratio >= cfg.whiteRatioThreshold;
                if (occupied[idx]) {
                    occupiedFound++;
                }
            }
        }

        List<List<Integer>> states = GridClustering.groupTouching(occupied, rows, cols);

        ClusterResult result = new ClusterResult();
        result.occupied = occupied;
        result.whiteRatio = whiteRatio;
        result.states = states;
        result.rows = rows;
        result.cols = cols;
        occupiedCount = occupiedFound;
        stateCount = states.size();

        List<Integer> best = null;
        for (List<Integer> state : states) {
            Point center = momentCentroid(mask, state, cols, rows, width, height, whiteRatio);
            result.groups.add(new StateGroup(state, center));
            if (best == null || state.size() > best.size()) {
                best = state;
            }
        }
        if (best == null || best.isEmpty()) {
            bestStateSize = 0;
            result.valid = false;
            return result;
        }

        bestStateSize = best.size();
        result.bestState = best;
        result.valid = true;
        result.centerPx = result.groups.get(0).centerPx;
        for (StateGroup group : result.groups) {
            if (group.cells == best) {
                result.centerPx = group.centerPx;
                break;
            }
        }
        return result;
    }

    private Point momentCentroid(Mat mask, List<Integer> best, int cols, int rows,
                                 int width, int height, double[] whiteRatio) {
        clusterMask.create(mask.rows(), mask.cols(), mask.type());
        clusterMask.setTo(new Scalar(0));
        for (int idx : best) {
            int r = idx / cols;
            int c = idx % cols;
            Rect roi = cellRect(c, r, width, height, cols, rows);
            Mat src = mask.submat(roi);
            Mat dst = clusterMask.submat(roi);
            src.copyTo(dst);
            src.release();
            dst.release();
        }

        Moments m = Imgproc.moments(clusterMask, true);
        if (m.get_m00() > 1e-6) {
            return new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00());
        }

        double sumX = 0;
        double sumY = 0;
        double sumW = 0;
        for (int idx : best) {
            int r = idx / cols;
            int c = idx % cols;
            Rect roi = cellRect(c, r, width, height, cols, rows);
            double w = Math.max(whiteRatio[idx], 1e-6);
            sumX += (roi.x + roi.width / 2.0) * w;
            sumY += (roi.y + roi.height / 2.0) * w;
            sumW += w;
        }
        return new Point(sumX / sumW, sumY / sumW);
    }

    private List<Ball> ballsNearGroup(List<Ball> found, StateGroup group, ClusterResult grid,
                                     int width, int height) {
        List<Ball> near = new ArrayList<Ball>();
        double searchR = cfg.ballSearchRadiusPx * (width / (double) LocalizationMath.CALIBRATION_WIDTH);
        for (Ball ball : found) {
            boolean close = Math.hypot(ball.center.x - group.centerPx.x,
                    ball.center.y - group.centerPx.y) <= searchR;
            boolean inState = GridClustering.cellContainsPoint(
                    group.cells, grid.rows, grid.cols, ball.center.x, ball.center.y, width, height);
            if (close || inState) {
                near.add(ball);
            }
        }
        return near;
    }

    private static Ball closestBall(List<Ball> near, Point center) {
        Ball closest = null;
        double closestDist = Double.MAX_VALUE;
        for (Ball ball : near) {
            double dist = Math.hypot(ball.center.x - center.x, ball.center.y - center.y);
            if (dist < closestDist) {
                closestDist = dist;
                closest = ball;
            }
        }
        return closest;
    }

    /**
     * Localize every occupied grid group, attach nearby balls, optionally
     * smooth X/Y, then number left-to-right.
     */
    private List<ClusterInfo> publishAllClusters(ClusterResult grid, List<Ball> found,
                                                int width, int height) {
        List<ClusterInfo> raw = new ArrayList<ClusterInfo>();
        int nearTotal = 0;
        for (StateGroup group : grid.groups) {
            List<Ball> near = ballsNearGroup(found, group, grid, width, height);
            Ball nearBall = closestBall(near, group.centerPx);
            if (nearBall != null && nearBall.z <= cfg.maxRangeInches) {
                nearTotal++;
            }
            Localized loc = localizeGroup(group.centerPx, nearBall, width, height);
            if (loc == null) {
                continue;
            }
            double circ = 0;
            double rad = 0;
            if (!near.isEmpty()) {
                for (Ball ball : near) {
                    circ += ball.circularity;
                    rad += ball.radiusPx;
                }
                circ /= near.size();
                rad /= near.size();
            }
            double[] xs = new double[near.size()];
            double[] ys = new double[near.size()];
            for (int i = 0; i < near.size(); i++) {
                xs[i] = near.get(i).center.x;
                ys[i] = near.get(i).center.y;
            }
            double tightness = GridClustering.tightness(xs, ys, Math.max(rad, 1));
            double confidence = EasyOBJDConfig.clamp(circ, 0, 1);
            if (near.isEmpty()) {
                confidence = 0.45;
            }
            if (loc.method == LocalizationMethod.FLOOR_PLANE) {
                confidence = Math.min(1.0, confidence + 0.08);
            }
            raw.add(new ClusterInfo(0, loc.x, loc.z, group.centerPx, group.cells,
                    LocalizationMath.planarRange(loc.x, loc.z), near.size(), circ, confidence, rad,
                    loc.method, near, tightness, Double.NaN, Double.NaN));
        }

        raw = smoothClusters(raw);
        raw.sort((a, b) -> Double.compare(a.centerPx.x, b.centerPx.x));

        List<ClusterInfo> numbered = new ArrayList<ClusterInfo>(raw.size());
        for (int i = 0; i < raw.size(); i++) {
            ClusterInfo src = raw.get(i);
            double fieldX = Double.NaN;
            double fieldY = Double.NaN;
            if (poseSet) {
                double[] field = LocalizationMath.cameraToField(
                        src.x, src.y, robotX, robotY, robotHeading,
                        cfg.cameraForwardOfCenter, cfg.cameraRightOfCenter, cfg.cameraYawRad());
                fieldX = field[0];
                fieldY = field[1];
            }
            numbered.add(src.withIdAndPose(i + 1, src.x, src.y, fieldX, fieldY));
        }

        ballsNearCluster = nearTotal;
        if (numbered.isEmpty()) {
            invalidateCluster();
            return numbered;
        }

        ClusterInfo left = numbered.get(0);
        clusterValid = true;
        clusterPixelX = left.centerPx.x;
        clusterPixelY = left.centerPx.y;
        clusterCamX = left.x;
        clusterCamZ = left.y;
        clusterUsedFloorPlane = left.localization == LocalizationMethod.FLOOR_PLANE;
        clusterRadiusPx = left.pixelRadius;
        clusterSizeRangeIn = left.ballCount > 0 && !left.balls.isEmpty()
                ? left.balls.get(0).z : Double.NaN;
        double[] robot = LocalizationMath.cameraToRobot(
                left.x, left.y, cfg.cameraForwardOfCenter, cfg.cameraRightOfCenter, cfg.cameraYawRad());
        clusterRobotX = robot[0];
        clusterRobotY = robot[1];
        if (poseSet) {
            FieldX.set(left.fieldX);
            FieldY.set(left.fieldY);
        } else {
            FieldX.set(Double.NaN);
            FieldY.set(Double.NaN);
        }
        return numbered;
    }

    /**
     * Floor-plane first when the ray hits inside max range; size-based
     * fallback when a nearby ball radius is available.
     */
    private Localized localizeGroup(Point centerPx, Ball nearBall, int width, int height) {
        double[] floorCam = LocalizationMath.pixelToFloor(
                centerPx.x, centerPx.y, width, height,
                cfg.cameraHeightInches, cfg.ballDiameterInches,
                cfg.focalLengthPixelsAt640, cfg.horizontalFovDegrees, cfg.cameraTiltDegrees);
        boolean usedFloor = floorCam != null && LocalizationMath.inRange(floorCam[2], cfg.maxRangeInches);
        if (usedFloor) {
            if (nearBall != null && !sizePlausible(nearBall.radiusPx, floorCam[2], width)) {
                // Keep the floor hit — the grid saw yellow — but prefer size if floor size is absurd.
                if (nearBall.z > 0 && nearBall.z <= cfg.maxRangeInches
                        && (nearBall.radiusPx < LocalizationMath.expectedRadiusPx(
                        cfg.ballDiameterInches, floorCam[2],
                        cfg.focalLengthPixelsAt640, cfg.horizontalFovDegrees, width)
                        * cfg.minExpectedSizeFrac * 0.5)) {
                    return Localized.size(nearBall.x, nearBall.y, nearBall.z);
                }
            }
            return Localized.floor(floorCam[0], floorCam[1], floorCam[2]);
        }
        if (nearBall == null) {
            return null;
        }
        if (nearBall.z <= 0 || nearBall.z > cfg.maxRangeInches) {
            return null;
        }
        return Localized.size(nearBall.x, nearBall.y, nearBall.z);
    }

    private boolean sizePlausible(double radiusPx, double rangeIn, int frameWidth) {
        double expected = LocalizationMath.expectedRadiusPx(
                cfg.ballDiameterInches, rangeIn,
                cfg.focalLengthPixelsAt640, cfg.horizontalFovDegrees, frameWidth);
        return radiusPx >= expected * cfg.minExpectedSizeFrac
                && radiusPx <= expected * cfg.maxExpectedSizeFrac;
    }

    private List<ClusterInfo> smoothClusters(List<ClusterInfo> incoming) {
        double a = cfg.smoothingAlpha;
        if (a <= 0 || a > 1) {
            rememberClusters(incoming);
            return incoming;
        }
        List<ClusterInfo> out = new ArrayList<ClusterInfo>(incoming.size());
        boolean[] used = new boolean[prevClusterXy.size()];
        for (ClusterInfo cluster : incoming) {
            int match = -1;
            double best = 12.0;
            for (int i = 0; i < prevClusterXy.size(); i++) {
                if (used[i]) {
                    continue;
                }
                double dist = Math.hypot(cluster.x - prevClusterXy.get(i)[0],
                        cluster.y - prevClusterXy.get(i)[1]);
                if (dist < best) {
                    best = dist;
                    match = i;
                }
            }
            double x = cluster.x;
            double y = cluster.y;
            if (match >= 0) {
                used[match] = true;
                x = a * cluster.x + (1.0 - a) * prevClusterXy.get(match)[0];
                y = a * cluster.y + (1.0 - a) * prevClusterXy.get(match)[1];
            }
            out.add(cluster.withIdAndPose(cluster.id, x, y, Double.NaN, Double.NaN));
        }
        rememberClusters(out);
        return out;
    }

    private void rememberClusters(List<ClusterInfo> clusters) {
        prevClusterXy.clear();
        for (ClusterInfo cluster : clusters) {
            prevClusterXy.add(new double[] { cluster.x, cluster.y });
        }
    }

    private void invalidateCluster() {
        clusterValid = false;
        clusterCamX = Double.NaN;
        clusterCamZ = Double.NaN;
        clusterRobotX = Double.NaN;
        clusterRobotY = Double.NaN;
        clusterSizeRangeIn = Double.NaN;
        clusterRadiusPx = Double.NaN;
        clusterUsedFloorPlane = false;
        FieldX.set(Double.NaN);
        FieldY.set(Double.NaN);
        ballsNearCluster = 0;
        ballSnapshot.set(Collections.<Ball>emptyList());
        clusterSnapshot.set(Collections.<ClusterInfo>emptyList());
        prevClusterXy.clear();
    }

    // ---- Ball / arc split ----

    private List<Ball> findBalls(Mat mask, int frameWidth, int frameHeight) {
        releaseContours(contourBuf);
        Imgproc.findContours(mask, contourBuf, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        double minRadius = minRadiusPx(frameWidth);
        double minArea = Math.PI * minRadius * minRadius * 0.45;
        double horizon = LocalizationMath.horizonImageY(
                cfg.cameraTiltDegrees, focalPx(frameWidth), frameHeight);

        List<Ball> found = new ArrayList<Ball>();
        for (MatOfPoint contour : contourBuf) {
            double area = Imgproc.contourArea(contour);
            if (area < minArea) {
                continue;
            }

            Point[] pts = contour.toArray();
            contour2fScratch.fromArray(pts);
            double perimeter = Imgproc.arcLength(contour2fScratch, true);
            Point circleCenter = new Point();
            Imgproc.minEnclosingCircle(contour2fScratch, circleCenter, circleRadiusBuf);
            if (perimeter <= 0) {
                continue;
            }

            double circularity = (4 * Math.PI * area) / (perimeter * perimeter);
            double fillRatio = area / (Math.PI * circleRadiusBuf[0] * circleRadiusBuf[0]);
            RotatedRect box = Imgproc.minAreaRect(contour2fScratch);
            double aw = box.size.width;
            double ah = box.size.height;
            double aspect = (aw < 1e-3 || ah < 1e-3)
                    ? Double.POSITIVE_INFINITY
                    : Math.max(aw, ah) / Math.min(aw, ah);

            boolean highQualitySingle = circleRadiusBuf[0] >= minRadius
                    && circularity >= cfg.singleBallMinCircularity
                    && fillRatio >= cfg.singleBallMinFill
                    && aspect <= cfg.maxAspectRatio
                    && isSolidBall(mask, contour, circleCenter, circleRadiusBuf[0]);

            if (highQualitySingle) {
                Ball single = makeBall(circleCenter, circleRadiusBuf[0], area, circularity,
                        fillRatio, frameWidth, frameHeight, horizon);
                if (single != null) {
                    found.add(single);
                    continue;
                }
            }

            boolean looseSingle = circleRadiusBuf[0] >= minRadius
                    && circularity >= cfg.minCircularity
                    && aspect <= cfg.maxAspectRatio
                    && !(circularity < cfg.singleBallMinCircularity && fillRatio < cfg.singleBallMinFill)
                    && isSolidBall(mask, contour, circleCenter, circleRadiusBuf[0]);
            if (looseSingle) {
                Ball single = makeBall(circleCenter, circleRadiusBuf[0], area, circularity,
                        fillRatio, frameWidth, frameHeight, horizon);
                if (single != null) {
                    found.add(single);
                    continue;
                }
            }

            for (CircleFit fit : splitArcs(pts, contour, mask, minRadius, frameWidth)) {
                Point fitCenter = new Point(fit.cx, fit.cy);
                if (!isSolidBall(mask, contour, fitCenter, fit.radius)) {
                    continue;
                }
                double fitArea = Math.PI * fit.radius * fit.radius;
                double fitCirc = 1.0 / (1.0 + fit.rms);
                Ball split = makeBall(fitCenter, fit.radius, fitArea, fitCirc,
                        1.0, frameWidth, frameHeight, horizon);
                if (split != null) {
                    found.add(split);
                }
            }
        }

        if (cfg.useHoughCircles) {
            addHoughBalls(mask, found, minRadius, frameWidth, frameHeight, horizon);
        }

        releaseContours(contourBuf);
        return dropOverlappingBalls(found);
    }

    private void addHoughBalls(Mat mask, List<Ball> found, double minRadius,
                               int frameWidth, int frameHeight, double horizon) {
        int minR = Math.max(3, (int) Math.round(minRadius));
        int maxR = Math.max(minR + 1, (int) Math.round(minRadius * 10));
        Imgproc.HoughCircles(mask, houghBuf, Imgproc.HOUGH_GRADIENT, 1.2,
                Math.max(minRadius * 1.8, 8), 120, 18, minR, maxR);
        if (houghBuf.empty() || houghBuf.cols() < 1) {
            return;
        }
        int n = houghBuf.cols();
        for (int i = 0; i < n; i++) {
            double[] circle = houghBuf.get(0, i);
            if (circle == null || circle.length < 3) {
                continue;
            }
            Point center = new Point(circle[0], circle[1]);
            double radius = circle[2];
            if (!isSolidBall(mask, null, center, radius)) {
                continue;
            }
            double area = Math.PI * radius * radius;
            Ball ball = makeBall(center, radius, area, 0.7, 0.7, frameWidth, frameHeight, horizon);
            if (ball != null) {
                found.add(ball);
            }
        }
    }

    /**
     * Fitted center must sit inside the yellow blob, the sampled center
     * pixel must be yellow, and the disk must be mostly yellow (rejects the
     * empty valley between two touching balls).
     */
    private boolean isSolidBall(Mat mask, MatOfPoint contour, Point center, double radius) {
        int ix = (int) Math.round(center.x);
        int iy = (int) Math.round(center.y);
        if (ix < 0 || iy < 0 || ix >= mask.cols() || iy >= mask.rows()) {
            return false;
        }
        if (contour != null) {
            contour2fScratch.fromArray(contour.toArray());
            if (Imgproc.pointPolygonTest(contour2fScratch, center, false) < 0) {
                return false;
            }
        }
        double[] px = mask.get(iy, ix);
        if (px == null || px[0] < 128) {
            return false;
        }
        return circleFillRatio(mask, center, radius) >= cfg.minCircleMaskFill;
    }

    private double circleFillRatio(Mat mask, Point center, double radius) {
        int r = Math.max(1, (int) Math.round(radius));
        int cx = (int) Math.round(center.x);
        int cy = (int) Math.round(center.y);
        int x0 = Math.max(0, cx - r);
        int y0 = Math.max(0, cy - r);
        int x1 = Math.min(mask.cols(), cx + r + 1);
        int y1 = Math.min(mask.rows(), cy + r + 1);
        if (x1 <= x0 || y1 <= y0) {
            return 0;
        }
        Mat src = mask.submat(new Rect(x0, y0, x1 - x0, y1 - y0));
        circleDisk.create(src.size(), src.type());
        circleDisk.setTo(new Scalar(0));
        Imgproc.circle(circleDisk, new Point(cx - x0, cy - y0), r, new Scalar(255), -1);
        Core.bitwise_and(src, circleDisk, circleAnd);
        int disk = Core.countNonZero(circleDisk);
        int yellow = Core.countNonZero(circleAnd);
        src.release();
        return disk == 0 ? 0 : yellow / (double) disk;
    }

    private List<Ball> dropOverlappingBalls(List<Ball> found) {
        found.sort((a, b) -> {
            int byRadius = Double.compare(b.radiusPx, a.radiusPx);
            if (byRadius != 0) {
                return byRadius;
            }
            return Double.compare(b.confidence, a.confidence);
        });
        List<Ball> kept = new ArrayList<Ball>();
        for (Ball ball : found) {
            boolean overlap = false;
            for (Ball other : kept) {
                double limit = cfg.ballOverlapFrac * (ball.radiusPx + other.radiusPx);
                if (Math.hypot(ball.center.x - other.center.x, ball.center.y - other.center.y) < limit) {
                    overlap = true;
                    break;
                }
            }
            if (!overlap) {
                kept.add(ball);
            }
        }
        return kept;
    }

    private Ball makeBall(Point center, double radiusPx, double area, double circularity,
                          double fillRatio, int frameWidth, int frameHeight, double horizon) {
        if (radiusPx < minRadiusPx(frameWidth)) {
            return null;
        }
        if (cfg.verticalBandFilter && center.y < horizon - 8) {
            return null;
        }
        double[] floor = LocalizationMath.pixelToFloor(
                center.x, center.y, frameWidth, frameHeight,
                cfg.cameraHeightInches, cfg.ballDiameterInches,
                cfg.focalLengthPixelsAt640, cfg.horizontalFovDegrees, cfg.cameraTiltDegrees);
        if (floor != null && LocalizationMath.inRange(floor[2], cfg.maxRangeInches)
                && !sizePlausible(radiusPx, floor[2], frameWidth)) {
            return null;
        }
        double[] cam = LocalizationMath.pixelToCamera(
                center.x, center.y, radiusPx, frameWidth, frameHeight,
                cfg.ballDiameterInches, cfg.focalLengthPixelsAt640,
                cfg.horizontalFovDegrees, cfg.cameraTiltDegrees);
        if (cam[2] > cfg.maxRangeInches || cam[2] <= 0) {
            return null;
        }
        double confidence = EasyOBJDConfig.clamp(circularity * Math.min(1.0, fillRatio / 0.85), 0, 1);
        return new Ball(center, radiusPx, area, circularity, cam[0], cam[1], cam[2], confidence);
    }

    /**
     * Slide a window around the contour and Kasa-fit a circle to each segment.
     * One well-fit arc = one visible ball. Several windows that agree vote
     * for the same ball; a second distinct center is a ball sitting behind / beside.
     *
     * <p>Weak (high-RMS) windows are discarded before voting. Window count is
     * capped on large contours so this stays cheap on the Control Hub.
     */
    private List<CircleFit> splitArcs(Point[] pts, MatOfPoint contour, Mat mask,
                                     double minRadius, int frameWidth) {
        int n = pts.length;
        if (n < 24) {
            return new ArrayList<CircleFit>();
        }

        int window = Math.max(20, n / 8);
        int step = Math.max(6, window / 3);
        int maxWindows = Math.max(6, cfg.maxArcWindows);
        if (n / step > maxWindows) {
            step = Math.max(6, (n + maxWindows - 1) / maxWindows);
        }
        double rmsLimit = cfg.arcRmsPx * (frameWidth / (double) LocalizationMath.CALIBRATION_WIDTH);
        List<CircleFit> votes = new ArrayList<CircleFit>();

        for (int i = 0; i < n; i += step) {
            CircleFit fit = fitCircleKasa(pts, i, window);
            if (fit == null || fit.radius < minRadius || fit.rms > rmsLimit) {
                continue;
            }
            if (!isSolidBall(mask, contour, new Point(fit.cx, fit.cy), fit.radius)) {
                continue;
            }
            votes.add(fit);
        }

        if (votes.size() > maxWindows) {
            Collections.sort(votes, (a, b) -> Double.compare(a.rms, b.rms));
            votes = new ArrayList<CircleFit>(votes.subList(0, maxWindows));
        }
        return mergeCircleVotes(votes);
    }

    private static CircleFit fitCircleKasa(Point[] pts, int start, int len) {
        int n = pts.length;
        if (len < 8) {
            return null;
        }
        double mx = 0;
        double my = 0;
        for (int i = 0; i < len; i++) {
            Point p = pts[(start + i) % n];
            mx += p.x;
            my += p.y;
        }
        mx /= len;
        my /= len;

        double suu = 0;
        double svv = 0;
        double suv = 0;
        double suuu = 0;
        double svvv = 0;
        double suuv = 0;
        double svvu = 0;
        for (int i = 0; i < len; i++) {
            Point p = pts[(start + i) % n];
            double u = p.x - mx;
            double v = p.y - my;
            double uu = u * u;
            double vv = v * v;
            suu += uu;
            svv += vv;
            suv += u * v;
            suuu += uu * u;
            svvv += vv * v;
            suuv += uu * v;
            svvu += vv * u;
        }

        double det = suu * svv - suv * suv;
        if (Math.abs(det) < 1e-6) {
            return null;
        }
        double uc = (svv * (suuu + svvu) - suv * (svvv + suuv)) / (2.0 * det);
        double vc = (suu * (svvv + suuv) - suv * (suuu + svvu)) / (2.0 * det);
        double radius = Math.sqrt(uc * uc + vc * vc + (suu + svv) / len);
        if (!(radius > 0) || Double.isNaN(radius)) {
            return null;
        }

        double cx = mx + uc;
        double cy = my + vc;
        double rms = 0;
        for (int i = 0; i < len; i++) {
            Point p = pts[(start + i) % n];
            double err = Math.hypot(p.x - cx, p.y - cy) - radius;
            rms += err * err;
        }
        rms = Math.sqrt(rms / len);
        return new CircleFit(cx, cy, radius, rms);
    }

    private List<CircleFit> mergeCircleVotes(List<CircleFit> votes) {
        List<CircleFit> merged = new ArrayList<CircleFit>();
        boolean[] used = new boolean[votes.size()];
        for (int i = 0; i < votes.size(); i++) {
            if (used[i]) {
                continue;
            }
            CircleFit seed = votes.get(i);
            double sx = 0;
            double sy = 0;
            double sr = 0;
            double sw = 0;
            int count = 0;
            for (int j = 0; j < votes.size(); j++) {
                if (used[j]) {
                    continue;
                }
                CircleFit other = votes.get(j);
                double maxR = Math.max(seed.radius, other.radius);
                if (Math.hypot(seed.cx - other.cx, seed.cy - other.cy) > cfg.arcCenterMergeFrac * maxR) {
                    continue;
                }
                if (Math.abs(seed.radius - other.radius) > cfg.arcRadiusMergeFrac * maxR) {
                    continue;
                }
                used[j] = true;
                double w = 1.0 / (1.0 + other.rms);
                sx += other.cx * w;
                sy += other.cy * w;
                sr += other.radius * w;
                sw += w;
                count++;
            }
            if (count >= cfg.minArcVotes && merged.size() < cfg.maxBallsPerBlob && sw > 0) {
                merged.add(new CircleFit(sx / sw, sy / sw, sr / sw, 0));
            }
        }
        return merged;
    }

    private double minRadiusPx(int frameWidth) {
        return LocalizationMath.minRadiusPx(
                cfg.minBallDiameterInches, cfg.maxRangeInches,
                cfg.focalLengthPixelsAt640, cfg.horizontalFovDegrees, frameWidth);
    }

    private double focalPx(int frameWidth) {
        return LocalizationMath.focalPx(
                cfg.focalLengthPixelsAt640, cfg.horizontalFovDegrees, frameWidth);
    }

    /** Camera (x right, z forward) → robot center {right, forward}, using static defaults. */
    static double[] cameraToRobot(double camX, double camZ) {
        return LocalizationMath.cameraToRobot(
                camX, camZ, CAMERA_FORWARD_OF_CENTER, CAMERA_RIGHT_OF_CENTER, CAMERA_YAW_RAD);
    }

    /** Camera (x right, z forward) → field {x, y}, using static defaults. */
    static double[] cameraToField(double camX, double camZ,
                                 double poseX, double poseY, double heading) {
        return LocalizationMath.cameraToField(
                camX, camZ, poseX, poseY, heading,
                CAMERA_FORWARD_OF_CENTER, CAMERA_RIGHT_OF_CENTER, CAMERA_YAW_RAD);
    }

    // ---- Preview ----

    private void drawPreview(Mat input, ClusterResult cluster, List<Ball> found,
                            List<ClusterInfo> clusters) {
        Imgproc.cvtColor(yellowMask, preview, Imgproc.COLOR_GRAY2BGR);
        if (preview.cols() != input.cols() || preview.rows() != input.rows()) {
            Imgproc.resize(preview, overlay, input.size(), 0, 0, Imgproc.INTER_NEAREST);
            overlay.copyTo(preview);
        }

        OverlayMode mode = cfg.overlayMode == null ? OverlayMode.FULL : cfg.overlayMode;
        if (mode == OverlayMode.MASK) {
            return;
        }

        double sx = preview.cols() / (double) yellowMask.cols();
        double sy = preview.rows() / (double) yellowMask.rows();
        int width = yellowMask.cols();
        int height = yellowMask.rows();
        boolean drawGrid = mode == OverlayMode.FULL || mode == OverlayMode.GRID;
        boolean drawBalls = mode == OverlayMode.FULL || mode == OverlayMode.BALLS
                || mode == OverlayMode.DISTANCES;
        boolean drawMarkers = mode == OverlayMode.FULL || mode == OverlayMode.DISTANCES;
        boolean drawLabels = mode == OverlayMode.FULL || mode == OverlayMode.DISTANCES;

        if (drawGrid) {
            int rows = cluster.rows;
            int cols = cluster.cols;
            int[] stateOf = new int[rows * cols];
            Arrays.fill(stateOf, -1);
            for (int s = 0; s < cluster.states.size(); s++) {
                for (int idx : cluster.states.get(s)) {
                    stateOf[idx] = s;
                }
            }

            overlay.create(preview.size(), preview.type());
            preview.copyTo(overlay);
            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < cols; c++) {
                    Rect roi = scaleRect(cellRect(c, r, width, height, cols, rows), sx, sy);
                    int idx = r * cols + c;
                    if (cluster.occupied[idx]) {
                        Imgproc.rectangle(overlay, roi, stateColor(stateOf[idx]), -1);
                    }
                }
            }
            Core.addWeighted(preview, 0.65, overlay, 0.35, 0, preview);
            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < cols; c++) {
                    Imgproc.rectangle(preview, scaleRect(cellRect(c, r, width, height, cols, rows), sx, sy),
                            new Scalar(80, 80, 80), 1);
                }
            }
        }

        if (drawBalls) {
            for (Ball ball : found) {
                Point drawC = new Point(ball.center.x * sx, ball.center.y * sy);
                Imgproc.circle(preview, drawC, (int) Math.round(ball.radiusPx * sx),
                        new Scalar(180, 180, 0), 2);
            }
        }

        if (drawMarkers) {
            for (ClusterInfo info : clusters) {
                Point drawCenter = new Point(info.centerPx.x * sx, info.centerPx.y * sy);
                Imgproc.drawMarker(preview, drawCenter, new Scalar(0, 0, 255),
                        Imgproc.MARKER_CROSS, 20, 2);
                String label = "#" + info.id;
                if (drawLabels) {
                    label = String.format(Locale.US, "#%d %.0f\"", info.id, info.y);
                }
                Imgproc.putText(preview, label,
                        new Point(drawCenter.x + 6, drawCenter.y - 6),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 255), 2);
            }
        }
    }

    private static Rect scaleRect(Rect roi, double sx, double sy) {
        return new Rect(
                (int) Math.round(roi.x * sx),
                (int) Math.round(roi.y * sy),
                (int) Math.round(roi.width * sx),
                (int) Math.round(roi.height * sy));
    }

    private static Scalar stateColor(int stateId) {
        Scalar[] palette = {
                new Scalar(180, 80, 0),
                new Scalar(0, 80, 180),
                new Scalar(120, 0, 160),
                new Scalar(0, 140, 140),
                new Scalar(80, 0, 200)
        };
        if (stateId < 0) {
            return new Scalar(40, 40, 40);
        }
        return palette[stateId % palette.length];
    }

    private static Rect cellRect(int col, int row, int width, int height, int cols, int rows) {
        int x0 = col * width / cols;
        int y0 = row * height / rows;
        int x1 = (col + 1) * width / cols;
        int y1 = (row + 1) * height / rows;
        return new Rect(x0, y0, x1 - x0, y1 - y0);
    }

    // ---- Public API ----

    /** Immutable snapshot from the last successful frame. */
    public List<Ball> getBalls() {
        return ballSnapshot.get();
    }

    /** Immutable snapshot from the last successful frame, left-to-right. */
    public List<ClusterInfo> getClusters() {
        return clusterSnapshot.get();
    }

    public int getClusterCount() {
        return clusterSnapshot.get().size();
    }

    public int getBallCount() {
        return ballSnapshot.get().size();
    }

    /**
     * Cluster that best matches a typical intake: closest piece by default.
     *
     * @return {@code null} when nothing is in range
     */
    public ClusterInfo getBestClusterForIntake() {
        return getBestClusterForIntake(IntakeHeuristic.CLOSEST);
    }

    public ClusterInfo getBestClusterForIntake(IntakeHeuristic heuristic) {
        List<ClusterInfo> clusters = getClusters();
        if (clusters.isEmpty()) {
            return null;
        }
        IntakeHeuristic rule = heuristic == null ? IntakeHeuristic.CLOSEST : heuristic;
        ClusterInfo best = clusters.get(0);
        for (ClusterInfo cluster : clusters) {
            switch (rule) {
                case LEFTMOST:
                    if (cluster.x < best.x) {
                        best = cluster;
                    }
                    break;
                case HIGHEST_CONFIDENCE:
                    if (cluster.confidence > best.confidence) {
                        best = cluster;
                    }
                    break;
                case CLOSEST:
                default:
                    if (cluster.rangeInches < best.rangeInches) {
                        best = cluster;
                    }
                    break;
            }
        }
        return best;
    }

    /** Cluster / ball counts. Pass {@code debug = true} for process time and errors. */
    public void addTelemetry(Telemetry telemetry) {
        addTelemetry(telemetry, false);
    }

    public void addTelemetry(Telemetry telemetry, boolean debug) {
        telemetry.addData("Clusters", getClusterCount());
        telemetry.addData("Balls", getBallCount());
        if (debug) {
            telemetry.addData("Process ms", "%.1f", getLastProcessTimeMs());
            telemetry.addData("Occupied cells", getOccupiedCount());
            telemetry.addData("Grid groups", getStateCount());
            telemetry.addData("Frames", getFrameCount());
            telemetry.addData("Overlay", cfg.overlayMode);
            telemetry.addData("Pose set", isPoseSet());
            if (lastError != null) {
                telemetry.addData("Error", lastError);
            }
        }
        for (ClusterInfo cluster : getClusters()) {
            telemetry.addLine(String.format(Locale.US,
                    "#%d  X=%.1f in  Y=%.1f in  %s  conf=%.2f  balls=%d",
                    cluster.id, cluster.x, cluster.y, cluster.localization,
                    cluster.confidence, cluster.ballCount));
            if (debug && isPoseSet()) {
                telemetry.addData("#" + cluster.id + " Field", "%.1f, %.1f",
                        cluster.fieldX, cluster.fieldY);
            }
        }
    }

    public boolean hasCluster() {
        return clusterValid;
    }

    public int getOccupiedCount() {
        return occupiedCount;
    }

    public int getStateCount() {
        return stateCount;
    }

    public int getBestStateSize() {
        return bestStateSize;
    }

    public int getBallsNearCluster() {
        return ballsNearCluster;
    }

    public double getClusterPixelX() {
        return clusterPixelX;
    }

    public double getClusterPixelY() {
        return clusterPixelY;
    }

    public double getClusterCamX() {
        return clusterCamX;
    }

    public double getClusterCamZ() {
        return clusterCamZ;
    }

    /** Inches right of the camera lens. + right, − left. */
    public double getClusterX() {
        return clusterCamX;
    }

    /** Inches forward of the camera lens (floor-plane when available). */
    public double getClusterY() {
        return clusterCamZ;
    }

    /** Size-based slant range of a ball on the leftmost cluster, inches. */
    public double getClusterSizeRangeIn() {
        return clusterSizeRangeIn;
    }

    public double getClusterRadiusPx() {
        return clusterRadiusPx;
    }

    public boolean usedFloorPlane() {
        return clusterUsedFloorPlane;
    }

    public double getFocalPxAt640() {
        return LocalizationMath.focalPx(
                cfg.focalLengthPixelsAt640, cfg.horizontalFovDegrees, LocalizationMath.CALIBRATION_WIDTH);
    }

    /** Inches right of robot center (lens offset + yaw applied). */
    public double getClusterRobotX() {
        return clusterRobotX;
    }

    /** Inches forward of robot center (lens offset + yaw applied). */
    public double getClusterRobotY() {
        return clusterRobotY;
    }

    public long getFrameCount() {
        return frameCount;
    }

    public double getLastProcessTimeMs() {
        return lastProcessTimeNs / 1_000_000.0;
    }

    public String getLastError() {
        return lastError;
    }

    public boolean isPoseSet() {
        return poseSet;
    }

    private static final class StateGroup {
        final List<Integer> cells;
        final Point centerPx;

        StateGroup(List<Integer> cells, Point centerPx) {
            this.cells = cells;
            this.centerPx = centerPx;
        }
    }

    private static final class ClusterResult {
        boolean valid;
        boolean[] occupied = new boolean[0];
        double[] whiteRatio = new double[0];
        List<List<Integer>> states = new ArrayList<List<Integer>>();
        List<StateGroup> groups = new ArrayList<StateGroup>();
        List<Integer> bestState;
        Point centerPx;
        int rows;
        int cols;
    }

    private static final class Localized {
        final double x;
        final double y;
        final double z;
        final LocalizationMethod method;

        private Localized(double x, double y, double z, LocalizationMethod method) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.method = method;
        }

        static Localized floor(double x, double y, double z) {
            return new Localized(x, y, z, LocalizationMethod.FLOOR_PLANE);
        }

        static Localized size(double x, double y, double z) {
            return new Localized(x, y, z, LocalizationMethod.SIZE_BASED);
        }
    }

    private static final class CircleFit {
        final double cx;
        final double cy;
        final double radius;
        final double rms;

        CircleFit(double cx, double cy, double radius, double rms) {
            this.cx = cx;
            this.cy = cy;
            this.radius = radius;
            this.rms = rms;
        }
    }
}
