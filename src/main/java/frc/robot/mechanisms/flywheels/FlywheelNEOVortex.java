package frc.robot.mechanisms.flywheels;

import com.revrobotics.CANSparkFlex;
import frc.robot.configurator.rev.REVConfigs;
import frc.robot.configurator.rev.REVConfigurator;
import frc.robot.controlLoops.velocity.VelocityControlLoop;

import static com.revrobotics.CANSparkLowLevel.*;

public class FlywheelNEOVortex extends Flywheel {
    private final CANSparkFlex canSparkFlex;

    public FlywheelNEOVortex(
            FlywheelConfigs flywheelConfigs,
            REVConfigs revConfigs,
            VelocityControlLoop velocityControlLoop) {
        super(velocityControlLoop);
        canSparkFlex = new CANSparkFlex(revConfigs.deviceId(), MotorType.kBrushless);
        REVConfigurator configurator = REVConfigurator.configure(canSparkFlex)
                .withIdleMode(revConfigs.mode())
                .withInverted(revConfigs.isInverted())
                .withAverageDepth(canSparkFlex.getEncoder(), revConfigs.averageDepth())
                .withMeasurementPeriod(canSparkFlex.getEncoder(), revConfigs.measurementPeriodMs())
                .withSmartCurrentLimit(revConfigs.smartCurrentLimit())
                .withPeriodicStatusFrame0Period(revConfigs.periodicStatusFrame1PeriodMs())
                .withPeriodicStatusFrame1Period(revConfigs.periodicStatusFrame2PeriodMs())
                .withPeriodicStatusFrame2Period(revConfigs.periodicStatusFrame2PeriodMs())
                .withPeriodicStatusFrame3Period(revConfigs.periodicStatusFrame3PeriodMs())
                .withPeriodicStatusFrame4Period(revConfigs.periodicStatusFrame4PeriodMs())
                .withPeriodicStatusFrame5Period(revConfigs.periodicStatusFrame5PeriodMs())
                .withPeriodicStatusFrame6Period(revConfigs.periodicStatusFrame6PeriodMs())
                .withVelocityConversionFactor(canSparkFlex.getEncoder(), flywheelConfigs.gearing());
    }

    @Override
    public void setInput(double input) {
        canSparkFlex.setVoltage(input);
    }

    @Override
    public void update() {
        current.mut_setMagnitude(canSparkFlex.getOutputCurrent());
        voltage.mut_setMagnitude(canSparkFlex.getAppliedOutput() * canSparkFlex.getBusVoltage());
        velocity.mut_setMagnitude(canSparkFlex.getEncoder().getVelocity());
    }
}
