package frc.robot.mechanisms.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public abstract class FlywheelPIDF extends Flywheel {

    private final SimpleMotorFeedforward simpleMotorFeedforward;
    private final PIDController pidController;
    private final double controlLoopPeriodSeconds;

    public FlywheelPIDF(FlywheelConfigs flywheelConfigs) {
        simpleMotorFeedforward = new SimpleMotorFeedforward(
                flywheelConfigs.kS(),
                flywheelConfigs.kV(),
                flywheelConfigs.kA());
        pidController = new PIDController(
                flywheelConfigs.kP(),
                0,
                0,
                flywheelConfigs.controlLoopPeriodSeconds());
        this.controlLoopPeriodSeconds = flywheelConfigs.controlLoopPeriodSeconds();
    }


    @Override
    public double getControlLoopOutput(
            Measure<Velocity<Angle>> currentVelocity,
            Measure<Velocity<Angle>> nextVelocity) {
        double voltageFF = simpleMotorFeedforward.calculate(
                currentVelocity.in(RadiansPerSecond),
                nextVelocity.in(RadiansPerSecond),
                controlLoopPeriodSeconds);
        double voltageFB = pidController.calculate(
                currentVelocity.in(RadiansPerSecond),
                nextVelocity.in(RadiansPerSecond));
        double totalVoltage = voltageFF + voltageFB;
        totalVoltage = MathUtil.clamp(totalVoltage, -12.0, 12.0);
        return totalVoltage;
    }


}

