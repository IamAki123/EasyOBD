/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.easyobd;

/**
 * @deprecated Use {@link EasyOBJD}.
 */
@Deprecated
public final class EasyOBD {
    private EasyOBD() {}

    public static EasyOBJDPipeline createPipeline() {
        return EasyOBJD.createPipeline();
    }

    public static EasyOBJDPipeline createPipeline(EasyOBJDConfig config) {
        return EasyOBJD.createPipeline(config);
    }
}
