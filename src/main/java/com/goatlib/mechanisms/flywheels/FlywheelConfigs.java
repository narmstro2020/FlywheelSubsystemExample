package com.goatlib.mechanisms.flywheels;

import edu.wpi.first.math.system.plant.DCMotor;

public record FlywheelConfigs(
        String name,
        DCMotor gearbox,
        double gearing,
        double controlLoopPeriodSeconds,
        double controlLoopPeriodOffsetSeconds,
        double updatePeriodSeconds,
        double updatePeriodOffsetSeconds,
        double kS,
        double kV,
        double kA,
        double kP
) {
}
