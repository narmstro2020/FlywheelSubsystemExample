package frc.robot.controlLoops;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import frc.robot.configs.PIDFConfigs;

import java.util.function.BiFunction;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class VelocityControlLoop {

    public static BiFunction<Measure<Velocity<Angle>>, Measure<Velocity<Angle>>, Measure<Voltage>> createSimpleMotorPIDF(
            PIDFConfigs pidfConfigs,
            double controlLoopPeriodSeconds
    ) {

        SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(
                pidfConfigs.kS(),
                pidfConfigs.kV(),
                pidfConfigs.kA());
        PIDController pidController = new PIDController(
                pidfConfigs.kP(),
                pidfConfigs.kI(),
                pidfConfigs.kD(),
                controlLoopPeriodSeconds);
        MutableMeasure<Voltage> controlLoopOutput = MutableMeasure.zero(Volts);

        return (currentVelocity, nextVelocity) -> {
            double voltageFF = simpleMotorFeedforward.calculate(
                    currentVelocity.in(RadiansPerSecond),
                    nextVelocity.in(RadiansPerSecond),
                    controlLoopPeriodSeconds);
            double voltageFB = pidController.calculate(
                    currentVelocity.in(RadiansPerSecond),
                    nextVelocity.in(RadiansPerSecond));
            double totalVoltage = voltageFF + voltageFB;
            totalVoltage = MathUtil.clamp(totalVoltage, -12.0, 12.0);
            controlLoopOutput.mut_setMagnitude(totalVoltage);
            return controlLoopOutput;
        };
    }

    private VelocityControlLoop() {
    }

}
