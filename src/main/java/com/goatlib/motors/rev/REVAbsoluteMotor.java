package com.goatlib.motors.rev;

import com.goatlib.configurator.rev.REVConfigs;
import com.revrobotics.*;

public class REVAbsoluteMotor extends REVMotor {

    private final AbsoluteEncoder absoluteEncoder;

    public REVAbsoluteMotor(REVConfigs revConfigs) {
        super(revConfigs);
        absoluteEncoder = canSparkBase.getAbsoluteEncoder();
        configurator
                .withAverageDepth(absoluteEncoder, revConfigs.averageDepth())
                .withConversionFactor(absoluteEncoder, revConfigs.motorToMechanismConversionFactor());
    }

    @Override
    public void update() {
        super.update();
        position.mut_setMagnitude(absoluteEncoder.getPosition());
        velocity.mut_setMagnitude(absoluteEncoder.getVelocity());
    }
}
