package org.firstinspires.ftc.easyobd;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Queue;

/**
 * Easy Object Detection pipeline: yellow HSV mask, 36-region clustering,
 * then circular / arc-split balls near the biggest cluster.
 *
 * <p>Camera-relative inches (no Pedro needed): {@link #getClusterX()} is right of the
 * lens (+ right), {@link #getClusterY()} is forward of the lens. Field-frame
 * {@code FieldX}/{@code FieldY} only update after {@link #setRobotPose}.
 */
public class EasyOBDCreation extends OpenCvPipeline {

    public static class Ball {
        public Point center;
        public double radiusPx;
        public double area;
        public double circularity;
        public double x;
        public double y;
        public double z;
    }

    /** One field-axis reading. {@link #bestClusterCenterpoint()} is NaN until a cluster is locked. */
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

    // ---- Grid (144 regions = 12x12). ----
    public static int GRID_ROWS = 12;
    public static int GRID_COLS = 12;
    /** Fraction of a cell that must be yellow/white to count as occupied. */
    public static double WHITE_RATIO_THRESHOLD = 0.12;
    /** How far (px at 640-wide) a circular ball may sit from the cluster centroid. */
    public static double BALL_SEARCH_RADIUS_PX = 220;

    // Wider than a tight pollen spec so glare / shade on the ball stays white.
    // D-pad U/D in EasyOBDTeleOp widens / tightens this live.
    public static Scalar yellowLower = new Scalar(21, 95, 85);
    public static Scalar yellowUpper = new Scalar(35, 255, 255);

    /** delta &gt; 0 widens yellow, delta &lt; 0 tightens. Safe to call from TeleOp. */
    public static void nudgeHsvRange(int delta) {
        if (delta == 0) {
            return;
        }
        yellowLower.val[0] = clamp(yellowLower.val[0] - delta, 0, 28);
        yellowUpper.val[0] = clamp(yellowUpper.val[0] + delta, 24, 50);
        yellowLower.val[1] = clamp(yellowLower.val[1] - 5 * delta, 20, 200);
        yellowLower.val[2] = clamp(yellowLower.val[2] - 5 * delta, 20, 200);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Official pollen diameter used for size-based range and the floor plane. */
    private static final double BALL_DIAMETER_INCHES = 2.8;
    /**
     * Hole / far-object cutoff. Apparent size of a 3in object at {@link #MAX_RANGE_INCHES}
     * is the smallest circle we will accept — smaller is a hole or a ball past 5ft.
     */
    public static double MIN_BALL_DIAMETER_INCHES = 3.0;
    /** Ignore balls / clusters farther than this (5 feet). */
    public static double MAX_RANGE_INCHES = 60.0;

    /**
     * Horizontal FOV at 640-wide. Used when {@link #FOCAL_LENGTH_PIXELS_AT_640} is 0.
     * 70.4° → ~454 px. The old 700 px placeholder assumed a ~49° lens and
     * reported ~1.5× too far (31 in tape → ~45–50 in).
     */
    public static double HORIZONTAL_FOV_DEGREES = 70.4;
    /**
     * 0 = derive from {@link #HORIZONTAL_FOV_DEGREES}. Set this after a tape
     * calibration: FOCAL = (2 * radiusPx * D) / 2.8 at 640-wide.
     */
    public static double FOCAL_LENGTH_PIXELS_AT_640 = 0;
    private static final int CALIBRATION_WIDTH = 640;
    /** Lens height above the floor. */
    public static double CAMERA_HEIGHT_INCHES = 19.0;
    /**
     * Downward pitch of the optical axis, degrees. 0 = level.
     * At 19 in up, a level camera cannot see a floor ball at 31 in — the
     * mount is angled down. Measure with a phone inclinometer on the housing.
     * If a ball at 31 in floor distance sits in the image center, start near
     * atan((19 - 1.4) / 31) ≈ 30°.
     */
    public static double CAMERA_TILT_DEGREES = 25.0;
    private static final double CAMERA_FORWARD_OF_CENTER = 0.75;
    private static final double CAMERA_RIGHT_OF_CENTER = 0.25;
    private static final double CAMERA_YAW_RAD = Math.toRadians(-4.0);

    /** 1.0 = full camera res. 0.5 = 320x240 process, 4x fewer pixels. */
    public static double PROCESS_SCALE = 1.0;

    private static final double SINGLE_BALL_MIN_CIRCULARITY = 0.75;
    private static final double SINGLE_BALL_MIN_FILL = 0.75;
    private static final double MIN_CIRCULARITY = 0.55;

    private static final double ARC_RMS_PX = 3.5;
    private static final double ARC_CENTER_MERGE_FRAC = 0.40;
    private static final double ARC_RADIUS_MERGE_FRAC = 0.30;
    private static final int MIN_ARC_VOTES = 2;
    private static final int MAX_BALLS_PER_BLOB = 8;
    /** Fitted circle must be this yellow inside (rejects the empty valley between two balls). */
    private static final double MIN_CIRCLE_MASK_FILL = 0.60;
    /** Drop a weaker circle whose center sits inside another accepted ball. */
    private static final double BALL_OVERLAP_FRAC = 0.55;

    private final Mat process = new Mat();
    private final Mat rgbMat = new Mat();
    private final Mat blurred = new Mat();
    private final Mat hsvMat = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat integral = new Mat();
    private final Mat clusterMask = new Mat();
    private final Mat preview = new Mat();
    private final Mat overlay = new Mat();
    private final Mat openKernel = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(5, 5));
    private final Mat closeKernel = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(9, 9));
    private final Mat circleDisk = new Mat();
    private final Mat circleAnd = new Mat();

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

    private final List<Ball> balls = new ArrayList<>();
    private final List<ClusterInfo> publishedClusters = new ArrayList<>();

    /** Push the current Pedro pose so pixel -> field conversion stays current. */
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
            Imgproc.GaussianBlur(src, blurred, new Size(5, 5), 0);
            toHsv(blurred, hsvMat);
            Core.inRange(hsvMat, yellowLower, yellowUpper, yellowMask);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, openKernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, closeKernel);
            fillExternalHoles(yellowMask);

            int pw = yellowMask.cols();
            int ph = yellowMask.rows();
            buildIntegral(yellowMask);

            List<Ball> found = findBalls(yellowMask, pw, ph);
            ClusterResult grid = findClusters(yellowMask);
            List<ClusterInfo> clusters = publishAllClusters(grid, found, pw, ph);

            synchronized (this) {
                balls.clear();
                balls.addAll(found);
                publishedClusters.clear();
                publishedClusters.addAll(clusters);
            }

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

    private Mat toProcessSize(Mat input) {
        if (PROCESS_SCALE >= 0.999) {
            return input;
        }
        Imgproc.resize(input, process, new Size(), PROCESS_SCALE, PROCESS_SCALE, Imgproc.INTER_AREA);
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

    private static void fillExternalHoles(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        Imgproc.drawContours(mask, contours, -1, new Scalar(255), -1);
    }

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
        int rows = GRID_ROWS;
        int cols = GRID_COLS;
        int width = mask.cols();
        int height = mask.rows();
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
                occupied[idx] = ratio >= WHITE_RATIO_THRESHOLD;
                if (occupied[idx]) {
                    occupiedFound++;
                }
            }
        }

        List<List<Integer>> states = groupTouching(occupied, rows, cols);

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

    private static List<List<Integer>> groupTouching(boolean[] occupied, int rows, int cols) {
        boolean[] visited = new boolean[occupied.length];
        List<List<Integer>> states = new ArrayList<>();
        int[] dr = {-1, 1, 0, 0};
        int[] dc = {0, 0, -1, 1};

        for (int start = 0; start < occupied.length; start++) {
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

    private Ball pickBallNearGroup(List<Ball> found, StateGroup group, ClusterResult grid,
                                  int width, int height) {
        Ball closest = null;
        double closestDist = Double.MAX_VALUE;
        double searchR = BALL_SEARCH_RADIUS_PX * (width / (double) CALIBRATION_WIDTH);

        for (Ball ball : found) {
            boolean near = Math.hypot(ball.center.x - group.centerPx.x,
                    ball.center.y - group.centerPx.y) <= searchR;
            boolean inState = cellContainsPoint(grid, group.cells, ball.center, width, height);
            if (!near && !inState) {
                continue;
            }
            double dist = Math.hypot(ball.center.x - group.centerPx.x,
                    ball.center.y - group.centerPx.y);
            if (dist < closestDist) {
                closestDist = dist;
                closest = ball;
            }
        }
        return closest;
    }

    private static boolean cellContainsPoint(ClusterResult grid, List<Integer> cells,
                                            Point p, int width, int height) {
        if (cells == null) {
            return false;
        }
        int col = (int) Math.min(grid.cols - 1, Math.max(0, p.x * grid.cols / width));
        int row = (int) Math.min(grid.rows - 1, Math.max(0, p.y * grid.rows / height));
        int idx = row * grid.cols + col;
        return cells.contains(idx);
    }

    private List<ClusterInfo> publishAllClusters(ClusterResult grid, List<Ball> found,
                                                int width, int height) {
        List<ClusterInfo> raw = new ArrayList<>();
        int nearTotal = 0;
        for (StateGroup group : grid.groups) {
            Ball nearBall = pickBallNearGroup(found, group, grid, width, height);
            if (nearBall != null && nearBall.z <= MAX_RANGE_INCHES) {
                nearTotal++;
            }
            CameraPoint cam = localizeGroup(group.centerPx, nearBall, width, height);
            if (cam == null) {
                continue;
            }
            raw.add(new ClusterInfo(0, cam.x, cam.z, group.centerPx, group.cells));
        }
        raw.sort((a, b) -> Double.compare(a.centerPx.x, b.centerPx.x));

        List<ClusterInfo> numbered = new ArrayList<>();
        for (int i = 0; i < raw.size(); i++) {
            ClusterInfo src = raw.get(i);
            numbered.add(new ClusterInfo(i + 1, src.x, src.y, src.centerPx, src.cells));
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
        double[] robot = cameraToRobot(left.x, left.y);
        clusterRobotX = robot[0];
        clusterRobotY = robot[1];
        if (poseSet) {
            double[] field = cameraToField(left.x, left.y, robotX, robotY, robotHeading);
            FieldX.set(field[0]);
            FieldY.set(field[1]);
        } else {
            FieldX.set(Double.NaN);
            FieldY.set(Double.NaN);
        }
        return numbered;
    }

    private CameraPoint localizeGroup(Point centerPx, Ball nearBall, int width, int height) {
        CameraPoint floorCam = pixelToFloor(centerPx, width, height);
        boolean usedFloor = floorCam != null && floorCam.z > 0 && floorCam.z <= MAX_RANGE_INCHES;
        if (usedFloor) {
            return floorCam;
        }
        if (nearBall == null) {
            return null;
        }
        CameraPoint sizeCam = pixelToCamera(centerPx, nearBall.radiusPx, width, height);
        if (sizeCam.z <= 0 || sizeCam.z > MAX_RANGE_INCHES) {
            return null;
        }
        return sizeCam;
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
        synchronized (this) {
            publishedClusters.clear();
        }
    }

    private List<Ball> findBalls(Mat mask, int frameWidth, int frameHeight) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        hierarchy.release();

        double minRadius = minRadiusPx(frameWidth);
        double minArea = Math.PI * minRadius * minRadius * 0.45;

        List<Ball> found = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < minArea) {
                continue;
            }

            Point[] pts = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(pts);
            double perimeter = Imgproc.arcLength(contour2f, true);
            Point circleCenter = new Point();
            float[] circleRadius = new float[1];
            Imgproc.minEnclosingCircle(contour2f, circleCenter, circleRadius);
            contour2f.release();
            if (perimeter <= 0) {
                continue;
            }

            double circularity = (4 * Math.PI * area) / (perimeter * perimeter);
            double fillRatio = area / (Math.PI * circleRadius[0] * circleRadius[0]);

            if (circleRadius[0] >= minRadius
                    && circularity >= MIN_CIRCULARITY
                    && !(circularity < SINGLE_BALL_MIN_CIRCULARITY && fillRatio < SINGLE_BALL_MIN_FILL)
                    && isSolidBall(mask, contour, circleCenter, circleRadius[0])) {
                Ball single = makeBall(circleCenter, circleRadius[0], area, circularity, frameWidth, frameHeight);
                if (single != null) {
                    found.add(single);
                    continue;
                }
            }

            // Merged / peanut blob: split into circular arcs (one arc per visible ball).
            for (CircleFit fit : splitArcs(pts, contour, mask, minRadius, frameWidth)) {
                Point fitCenter = new Point(fit.cx, fit.cy);
                if (!isSolidBall(mask, contour, fitCenter, fit.radius)) {
                    continue;
                }
                double fitArea = Math.PI * fit.radius * fit.radius;
                Ball split = makeBall(fitCenter, fit.radius, fitArea,
                        1.0 / (1.0 + fit.rms), frameWidth, frameHeight);
                if (split != null) {
                    found.add(split);
                }
            }
        }
        return dropOverlappingBalls(found);
    }

    /**
     * The empty notch between two touching balls is itself a circular arc.
     * Reject it: the fitted center must sit inside the yellow blob, and the
     * disk must be mostly yellow (a valley circle is mostly background).
     */
    private boolean isSolidBall(Mat mask, MatOfPoint contour, Point center, double radius) {
        int ix = (int) Math.round(center.x);
        int iy = (int) Math.round(center.y);
        if (ix < 0 || iy < 0 || ix   >= mask.cols() || iy >= mask.rows()) {
            return false;
        }
        if (contour != null) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double inside = Imgproc.pointPolygonTest(contour2f, center, false);
            contour2f.release();
            if (inside < 0) {
                return false;
            }
        }
        double[] px = mask.get(iy, ix);
        if (px == null || px[0] < 128) {
            return false;
        }
        return circleFillRatio(mask, center, radius) >= MIN_CIRCLE_MASK_FILL;
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

    private static List<Ball> dropOverlappingBalls(List<Ball> found) {
        found.sort((a, b) -> Double.compare(b.radiusPx, a.radiusPx));
        List<Ball> kept = new ArrayList<>();
        for (Ball ball : found) {
            boolean overlap = false;
            for (Ball other : kept) {
                double limit = BALL_OVERLAP_FRAC * (ball.radiusPx + other.radiusPx);
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
                          int frameWidth, int frameHeight) {
        if (radiusPx < minRadiusPx(frameWidth)) {
            return null;
        }
        CameraPoint cam = pixelToCamera(center, radiusPx, frameWidth, frameHeight);
        if (cam.z > MAX_RANGE_INCHES) {
            return null;
        }
        Ball ball = new Ball();
        ball.center = center;
        ball.radiusPx = radiusPx;
        ball.area = area;
        ball.circularity = circularity;
        ball.x = cam.x;
        ball.y = cam.y;
        ball.z = cam.z;
        return ball;
    }

    /**
     * Slide a window around the contour and Kasa-fit a circle to each segment.
     * One well-fit arc = one visible ball. Several windows that agree vote
     * for the same ball; a second distinct center is a ball sitting behind / beside.
     */
    private List<CircleFit> splitArcs(Point[] pts, MatOfPoint contour, Mat mask,
                                     double minRadius, int frameWidth) {
        int n = pts.length;
        if (n < 24) {
            return new ArrayList<>();
        }

        int window = Math.max(20, n / 8);
        int step = Math.max(6, window / 3);
        double rmsLimit = ARC_RMS_PX * (frameWidth / (double) CALIBRATION_WIDTH);
        List<CircleFit> votes = new ArrayList<>();

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

    private static List<CircleFit> mergeCircleVotes(List<CircleFit> votes) {
        List<CircleFit> merged = new ArrayList<>();
        boolean[] used = new boolean[votes.size()];
        for (int i = 0; i < votes.size(); i++) {
            if (used[i]) {
                continue;
            }
            CircleFit seed = votes.get(i);
            double sx = 0;
            double sy = 0;
            double sr = 0;
            int count = 0;
            for (int j = 0; j < votes.size(); j++) {
                if (used[j]) {
                    continue;
                }
                CircleFit other = votes.get(j);
                double maxR = Math.max(seed.radius, other.radius);
                if (Math.hypot(seed.cx - other.cx, seed.cy - other.cy) > ARC_CENTER_MERGE_FRAC * maxR) {
                    continue;
                }
                if (Math.abs(seed.radius - other.radius) > ARC_RADIUS_MERGE_FRAC * maxR) {
                    continue;
                }
                used[j] = true;
                sx += other.cx;
                sy += other.cy;
                sr += other.radius;
                count++;
            }
            if (count >= MIN_ARC_VOTES && merged.size() < MAX_BALLS_PER_BLOB) {
                merged.add(new CircleFit(sx / count, sy / count, sr / count, 0));
            }
        }
        return merged;
    }

    private static double minRadiusPx(int frameWidth) {
        double focal = focalPx(frameWidth);
        return (MIN_BALL_DIAMETER_INCHES * focal) / (2.0 * MAX_RANGE_INCHES);
    }

    private static double focalPx(int frameWidth) {
        if (FOCAL_LENGTH_PIXELS_AT_640 > 0) {
            return FOCAL_LENGTH_PIXELS_AT_640 * (frameWidth / (double) CALIBRATION_WIDTH);
        }
        return (frameWidth / 2.0) / Math.tan(Math.toRadians(HORIZONTAL_FOV_DEGREES / 2.0));
    }

    /**
     * Size-based pinhole: Z from known ball diameter vs apparent radius.
     * This is optical-axis depth, then rotated by downward pitch into level axes.
     */
    private CameraPoint pixelToCamera(Point center, double radiusPx, int frameWidth, int frameHeight) {
        double focal = focalPx(frameWidth);
        double zCam = (BALL_DIAMETER_INCHES * focal) / (radiusPx * 2.0);

        double dx = center.x - frameWidth / 2.0;
        double dy = center.y - frameHeight / 2.0;
        double xCam = zCam * dx / focal;
        double yUpCam = -zCam * dy / focal;

        return tiltToLevel(xCam, yUpCam, zCam);
    }

    /**
     * Floor-plane intersection using camera height + downward pitch.
     * Does not use ball size — better for tape-along-the-floor when the
     * lens is 19 in up. Returns null if the ray never hits the ball plane.
     */
    private CameraPoint pixelToFloor(Point center, int frameWidth, int frameHeight) {
        double focal = focalPx(frameWidth);
        double dx = center.x - frameWidth / 2.0;
        double dy = center.y - frameHeight / 2.0;
        CameraPoint dir = tiltToLevel(dx, -dy, focal);

        double planeY = BALL_DIAMETER_INCHES / 2.0 - CAMERA_HEIGHT_INCHES;
        if (dir.y >= -1e-6) {
            return null;
        }
        double t = planeY / dir.y;
        if (t <= 0) {
            return null;
        }
        CameraPoint hit = new CameraPoint();
        hit.x = t * dir.x;
        hit.y = planeY;
        hit.z = t * dir.z;
        return hit;
    }

    /** Positive {@link #CAMERA_TILT_DEGREES} = optical axis pitched down. */
    private static CameraPoint tiltToLevel(double xCam, double yUpCam, double zCam) {
        double tilt = Math.toRadians(CAMERA_TILT_DEGREES);
        CameraPoint cam = new CameraPoint();
        cam.x = xCam;
        cam.z = zCam * Math.cos(tilt) + yUpCam * Math.sin(tilt);
        cam.y = -zCam * Math.sin(tilt) + yUpCam * Math.cos(tilt);
        return cam;
    }

    /** Camera (x right, z forward) -> robot center {right, forward}, inches. */
    static double[] cameraToRobot(double camX, double camZ) {
        double robotForward = camZ * Math.cos(CAMERA_YAW_RAD) + camX * Math.sin(CAMERA_YAW_RAD)
                + CAMERA_FORWARD_OF_CENTER;
        double robotRight = -camZ * Math.sin(CAMERA_YAW_RAD) + camX * Math.cos(CAMERA_YAW_RAD)
                + CAMERA_RIGHT_OF_CENTER;
        return new double[] { robotRight, robotForward };
    }

    /**
     * Camera (x right, z forward) -> robot center -> Pedro field {x, y}.
     * Same yaw / offset convention as AprilTagLocalize + AprilTagAlignment.
     */
    static double[] cameraToField(double camX, double camZ,
                                 double poseX, double poseY, double heading) {
        double[] robot = cameraToRobot(camX, camZ);
        double robotRight = robot[0];
        double robotForward = robot[1];

        double fieldX = poseX + robotForward * Math.cos(heading) + robotRight * Math.sin(heading);
        double fieldY = poseY + robotForward * Math.sin(heading) - robotRight * Math.cos(heading);
        return new double[] { fieldX, fieldY };
    }

    private void drawPreview(Mat input, ClusterResult cluster, List<Ball> found,
                            List<ClusterInfo> clusters) {
        Imgproc.cvtColor(yellowMask, preview, Imgproc.COLOR_GRAY2BGR);
        if (preview.cols() != input.cols() || preview.rows() != input.rows()) {
            Imgproc.resize(preview, overlay, input.size(), 0, 0, Imgproc.INTER_NEAREST);
            overlay.copyTo(preview);
        }

        double sx = preview.cols() / (double) yellowMask.cols();
        double sy = preview.rows() / (double) yellowMask.rows();
        int width = yellowMask.cols();
        int height = yellowMask.rows();
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

        for (Ball ball : found) {
            Point drawC = new Point(ball.center.x * sx, ball.center.y * sy);
            Imgproc.circle(preview, drawC, (int) Math.round(ball.radiusPx * sx),
                    new Scalar(180, 180, 0), 2);
        }

        for (ClusterInfo info : clusters) {
            Point drawCenter = new Point(info.centerPx.x * sx, info.centerPx.y * sy);
            Imgproc.drawMarker(preview, drawCenter, new Scalar(0, 0, 255),
                    Imgproc.MARKER_CROSS, 20, 2);
            Imgproc.putText(preview, "#" + info.id,
                    new Point(drawCenter.x + 6, drawCenter.y - 6),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 255), 2);
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

    public synchronized List<Ball> getBalls() {
        return new ArrayList<>(balls);
    }

    public synchronized List<ClusterInfo> getClusters() {
        return new ArrayList<>(publishedClusters);
    }

    public synchronized int getClusterCount() {
        return publishedClusters.size();
    }

    public synchronized int getBallCount() {
        return balls.size();
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

    /** Size-based slant range, inches — cross-check only. */
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
        return focalPx(CALIBRATION_WIDTH);
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
        List<List<Integer>> states = new ArrayList<>();
        List<StateGroup> groups = new ArrayList<>();
        List<Integer> bestState;
        Point centerPx;
        int rows;
        int cols;
    }

    private static final class CameraPoint {
        double x;
        double y;
        double z;
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
