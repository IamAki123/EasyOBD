package org.firstinspires.ftc.easyobd;

/**
 * @deprecated Use {@link EasyOBDPipeline}. Kept so 0.1 call sites compile.
 */
@Deprecated
public class EasyOBDCreation extends EasyOBDPipeline {
    public EasyOBDCreation() {
        super();
    }

    public EasyOBDCreation(EasyOBDConfig config) {
        super(config);
    }
}
