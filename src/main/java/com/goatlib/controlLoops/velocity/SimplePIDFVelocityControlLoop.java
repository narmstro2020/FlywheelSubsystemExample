package com.goatlib.controlLoops.velocity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.goatlib.mechanisms.SimpleMotorConfigs;

public class SimplePIDFVelocityControlLoop implements VelocityControlLoop {

    private final SimpleMotorFeedforward simpleMotorFeedforward;
    private final PIDController pidController;
    private final double controlLoopPeriodSeconds;

    public SimplePIDFVelocityControlLoop(SimpleMotorConfigs flywheelConfigs) {
        simpleMotorFeedforward = new SimpleMotorFeedforward(
                flywheelConfigs.kS(),
                flywheelConfigs.kV(),
                flywheelConfigs.kA());
        pidController = new PIDController(
                flywheelConfigs.kP(),
                flywheelConfigs.kI(),
                flywheelConfigs.kD(),
                flywheelConfigs.controlLoopPeriodSeconds());
        this.controlLoopPeriodSeconds = flywheelConfigs.controlLoopPeriodSeconds();
    }

    @Override
    public double getOutput(double currentVelocity, double nextVelocity) {
        double voltageFF = simpleMotorFeedforward.calculate(
                currentVelocity,
                nextVelocity,
                controlLoopPeriodSeconds);
        double voltageFB = pidController.calculate(
                currentVelocity,
                nextVelocity);
        double totalVoltage = voltageFF + voltageFB;
        totalVoltage = MathUtil.clamp(totalVoltage, -12.0, 12.0);
        return totalVoltage;
    }
}
