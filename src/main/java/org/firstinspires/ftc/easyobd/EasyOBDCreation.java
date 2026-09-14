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
 * @deprecated Use {@link EasyOBJDPipeline}. Kept so 0.1 / 0.2 call sites compile.
 */
@Deprecated
public class EasyOBDCreation extends EasyOBJDPipeline {
    public EasyOBDCreation() {
        super();
    }

    public EasyOBDCreation(EasyOBJDConfig config) {
        super(config);
    }
}
