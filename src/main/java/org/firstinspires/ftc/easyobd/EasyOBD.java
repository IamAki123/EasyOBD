package org.firstinspires.ftc.easyobd;

/**
 * Factory for the EasyOBD yellow-cluster pipeline.
 *
 * <pre>
 * EasyOBDConfig config = EasyOBDConfig.builder()
 *         .camera(19, 25, 70.4)
 *         .processWidth(320)
 *         .build();
 * EasyOBDPipeline pipeline = EasyOBD.createPipeline(config);
 * webcam.setPipeline(pipeline);
 * </pre>
 */
public final class EasyOBD {
    private EasyOBD() {}

    public static EasyOBDPipeline createPipeline() {
        return new EasyOBDPipeline();
    }

    public static EasyOBDPipeline createPipeline(EasyOBDConfig config) {
        return new EasyOBDPipeline(config);
    }
}
