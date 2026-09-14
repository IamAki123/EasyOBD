/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobd;

/** Team-specific pick for {@link EasyOBJDPipeline#getBestClusterForIntake}. */
public enum IntakeHeuristic {
    /** Smallest planar range from the lens (closest game piece). */
    CLOSEST,
    /** Smallest camera X (leftmost in the image / to the robot's left). */
    LEFTMOST,
    /** Highest combined circularity / fill / localization confidence. */
    HIGHEST_CONFIDENCE
}
