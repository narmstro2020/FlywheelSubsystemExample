package com.goatlib.motors.rev;

import com.goatlib.configurator.rev.REVConfigs;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class REVRelativeMotor extends REVMotor {

    private final RelativeEncoder relativeEncoder;

    public REVRelativeMotor(REVConfigs revConfigs) {
        super(revConfigs);
        relativeEncoder = canSparkBase.getEncoder();

    }

    public REVRelativeMotor(REVConfigs revConfigs, int countsPerRev) {
        super(revConfigs);

        if (canSparkBase instanceof CANSparkFlex) {
            relativeEncoder = ((CANSparkFlex) canSparkBase).getExternalEncoder(countsPerRev);
        } else {
            relativeEncoder = ((CANSparkMax) canSparkBase).getAlternateEncoder(countsPerRev);
        }
        configurator
                .withAverageDepth(relativeEncoder, revConfigs.averageDepth())
                .withMeasurementPeriod(relativeEncoder, revConfigs.measurementPeriodMs())
                .withConversionFactor(relativeEncoder, revConfigs.motorToMechanismConversionFactor());
    }

    @Override
    public void update() {
        super.update();
        position.mut_setMagnitude(relativeEncoder.getPosition());
        velocity.mut_setMagnitude(relativeEncoder.getVelocity());
    }

}
