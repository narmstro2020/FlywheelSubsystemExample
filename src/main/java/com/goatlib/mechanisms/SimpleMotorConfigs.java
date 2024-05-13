package com.goatlib.mechanisms;

import edu.wpi.first.math.system.plant.DCMotor;

public record SimpleMotorConfigs(
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
        double kP,
        double kI,
        double kD) {
}
