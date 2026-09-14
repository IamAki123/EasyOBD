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
 * Factory for the EasyOBJD game-piece pipeline.
 *
 * <pre>
 * EasyOBJDConfig config = EasyOBJDConfig.builder()
 *         .camera(19, 25, 70.4)
 *         .hsv(21, 95, 85, 35, 255, 255)
 *         .ballSize(2.8)
 *         .processWidth(320)
 *         .build();
 * EasyOBJDPipeline pipeline = EasyOBJD.createPipeline(config);
 * webcam.setPipeline(pipeline);
 * </pre>
 */
public final class EasyOBJD {
    private EasyOBJD() {}

    public static EasyOBJDPipeline createPipeline() {
        return new EasyOBJDPipeline();
    }

    public static EasyOBJDPipeline createPipeline(EasyOBJDConfig config) {
        return new EasyOBJDPipeline(config);
    }
}
