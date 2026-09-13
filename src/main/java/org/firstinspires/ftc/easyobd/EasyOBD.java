package org.firstinspires.ftc.easyobd;

/**
 * Factory for the EasyOBD yellow-cluster pipeline.
 *
 * <pre>
 * EasyOBDCreation pipeline = EasyOBD.createPipeline();
 * webcam.setPipeline(pipeline);
 * </pre>
 */
public final class EasyOBD {
    private EasyOBD() {}

    public static EasyOBDCreation createPipeline() {
        return new EasyOBDCreation();
    }
}
