package org.firstinspires.ftc.easyobd;

/**
 * How a cluster's camera-relative X/Y was computed.
 *
 * <p>{@link #FLOOR_PLANE} is preferred whenever the pixel ray intersects the
 * ball-center plane (camera height + downward tilt) inside
 * {@link EasyOBDConfig#maxRangeInches}. {@link #SIZE_BASED} is the fallback
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
