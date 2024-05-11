package frc.robot.subsystems;


import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConfigs;
import frc.robot.configs.REVConfigurator;

import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

import static com.revrobotics.CANSparkLowLevel.*;
import static edu.wpi.first.units.Units.*;

public class FlywheelSubsystem extends SubsystemBase {

    public static FlywheelSubsystem createNEOVortex(
            Constants.REVMotorFlywheelConfigs revMotorConfig,
            FlywheelConfigs flywheelConfigs,
            BiFunction<Measure<Velocity<Angle>>, Measure<Velocity<Angle>>, Measure<Voltage>> controlLoop,
            Function<Runnable, BiConsumer<Double, Double>> addPeriodicMethod) {
        CANSparkFlex canSparkFlex = new CANSparkFlex(revMotorConfig.deviceId, MotorType.kBrushless);
        REVConfigurator configurator = REVConfigurator.configure(canSparkFlex)
                .withInverted(revMotorConfig.isInverted)
                .withAverageDepth(canSparkFlex.getEncoder(), revMotorConfig.averageDepth)
                .withMeasurementPeriod(canSparkFlex.getEncoder(), revMotorConfig.measurementPeriodMs)
                .withSmartCurrentLimit(revMotorConfig.smartCurrentLimit)
                .withPeriodicStatusFrame0Period(revMotorConfig.periodicStatusFrame1PeriodMs)
                .withPeriodicStatusFrame1Period(revMotorConfig.periodicStatusFrame2PeriodMs)
                .withPeriodicStatusFrame2Period(revMotorConfig.periodicStatusFrame2PeriodMs)
                .withPeriodicStatusFrame3Period(revMotorConfig.periodicStatusFrame3PeriodMs)
                .withPeriodicStatusFrame4Period(revMotorConfig.periodicStatusFrame4PeriodMs)
                .withPeriodicStatusFrame5Period(revMotorConfig.periodicStatusFrame5PeriodMs)
                .withPeriodicStatusFrame6Period(revMotorConfig.periodicStatusFrame6PeriodMs);
        MutableMeasure<Current> current = MutableMeasure.zero(Amps);
        MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
        MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RadiansPerSecond);
        return new FlywheelSubsystem(
                flywheelConfigs.name,
                current,
                voltage,
                velocity,
                (inputVoltage) -> canSparkFlex.setVoltage(inputVoltage.in(Volts)),
                () -> {
                    current.mut_setMagnitude(canSparkFlex.getOutputCurrent());
                    voltage.mut_setMagnitude(canSparkFlex.getAppliedOutput() * canSparkFlex.getBusVoltage());
                    velocity.mut_setMagnitude(canSparkFlex.getEncoder().getVelocity());
                },
                controlLoop,
                flywheelConfigs.controlLoopPeriodSeconds,
                flywheelConfigs.controlLoopPeriodOffsetSeconds,
                flywheelConfigs.updaterPeriodSeconds,
                flywheelConfigs.updaterPeriodOffsetSeconds,
                addPeriodicMethod

        );
    }

    public static FlywheelSubsystem createSimulated(
            FlywheelConfigs flywheelConfigs,
            BiFunction<Measure<Velocity<Angle>>, Measure<Velocity<Angle>>, Measure<Voltage>> controlLoop,
            Function<Runnable, BiConsumer<Double, Double>> addPeriodicMethod) {

        MutableMeasure<Current> current = MutableMeasure.zero(Amps);
        MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
        MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RadiansPerSecond);
        LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(
                flywheelConfigs.kV,
                flywheelConfigs.kA);
        SimFlywheel simFlywheel = new SimFlywheel(
                plant,
                flywheelConfigs.gearbox,
                flywheelConfigs.gearing);

        return new FlywheelSubsystem(
                flywheelConfigs.name,
                current,
                voltage,
                velocity,
                (inputVoltage) -> simFlywheel.setInputVoltage(inputVoltage.in(Volts)),
                () -> {
                    current.mut_setMagnitude(simFlywheel.getCurrentDrawAmps());
                    voltage.mut_setMagnitude(simFlywheel.getInputVoltage());
                    velocity.mut_setMagnitude(simFlywheel.getAngularVelocityRadPerSec());
                    simFlywheel.update(flywheelConfigs.updaterPeriodSeconds);
                },
                controlLoop,
                flywheelConfigs.controlLoopPeriodSeconds,
                flywheelConfigs.controlLoopPeriodOffsetSeconds,
                flywheelConfigs.updaterPeriodSeconds,
                flywheelConfigs.updaterPeriodOffsetSeconds,
                addPeriodicMethod);
    }


    private final Measure<Current> current;
    private final Measure<Voltage> voltage;
    private final Measure<Velocity<Angle>> velocity;
    private final MutableMeasure<Velocity<Angle>> velocitySetpoint = MutableMeasure.zero(RadiansPerSecond);
    private final SysIdRoutine sysIdRoutine;
    private boolean sysIdActive = false;


    private FlywheelSubsystem(
            String name,
            Measure<Current> current,
            Measure<Voltage> voltage,
            Measure<Velocity<Angle>> velocity,
            Consumer<Measure<Voltage>> voltageConsumer,
            Runnable updater,
            BiFunction<Measure<Velocity<Angle>>, Measure<Velocity<Angle>>, Measure<Voltage>> controlLoop,
            double controlLoopPeriodSeconds,
            double controlLoopPeriodOffsetSeconds,
            double updaterPeriodSeconds,
            double updaterPeriodOffsetSeconds,
            Function<Runnable, BiConsumer<Double, Double>> addPeriodicMethod) {
        this.current = current;
        this.voltage = voltage;
        this.velocity = velocity;

        addPeriodicMethod.apply(updater).accept(updaterPeriodSeconds, updaterPeriodOffsetSeconds);

        addPeriodicMethod.apply(
                        () -> {
                            if (!sysIdActive) {
                                voltageConsumer.accept(
                                        controlLoop.apply(velocity, velocitySetpoint)
                                );
                            }
                        })
                .accept(controlLoopPeriodSeconds, controlLoopPeriodOffsetSeconds);

        sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motor(s).
                        voltageConsumer,
                        // Tell SysId how to record a frame of data for each motor on the mechanism being
                        // characterized.
                        log -> {
                            // Record a frame for the shooter motor.
                            log.motor("flywheel")
                                    .voltage(voltage)
                                    .angularVelocity(velocity);
                        },
                        // Tell SysId to make generated commands require this subsystem, suffix test state in
                        // WPILog with this subsystem's name ("shooter")
                        this,
                        name));
    }

    public Trigger createAtSetpointTrigger(Measure<Velocity<Angle>> setpoint, Measure<Velocity<Angle>> tolerance) {
        return new Trigger(() -> MathUtil.isNear(setpoint.in(RadiansPerSecond), velocity.in(RadiansPerSecond), tolerance.in(RadiansPerSecond)));
    }


    public Command createSetVelocityCommand(Measure<Velocity<Angle>> velocity) {
        return Commands.sequence(
                        runOnce(() -> sysIdActive = false),
                        run(() -> velocitySetpoint.mut_setMagnitude(velocity.in(RadiansPerSecond))))
                .withName(String.format("Velocity set to %s rad/s", velocity.in(RadiansPerSecond)));
    }

    /**
     * Returns a command that will execute a quasistatic test in the forward direction.
     */
    public Command sysIdQuasistaticForward() {
        return Commands.sequence(
                        runOnce(() -> sysIdActive = true),
                        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward))
                .withName("sysIdQuasiForward");
    }

    /**
     * Returns a command that will execute a quasistatic test in the reverse direction.
     */
    public Command sysIdQuasistaticReverse() {
        return Commands.sequence(
                        runOnce(() -> sysIdActive = true),
                        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
                .withName("sysIdQuasiReverse");
    }

    /**
     * Returns a command that will execute a dynamic test in the forward direction.
     */
    public Command sysIdDynamicForward() {
        return Commands.sequence(
                        runOnce(() -> sysIdActive = true),
                        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
                .withName("sysIdDynamicForward");
    }

    /**
     * Returns a command that will execute a dynamic test in the reverse direction.
     */
    public Command sysIdDynamicReverse() {
        return Commands.sequence(
                        runOnce(() -> sysIdActive = true),
                        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse))
                .withName("sysIdDynamicReverse");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(
                "Velocity (rpm)",
                () -> velocity.in(RPM),
                null);
        builder.addDoubleProperty(
                "Velocity Setpoint (rpm)",
                () -> velocitySetpoint.in(RPM),
                null);
        builder.addDoubleProperty(
                "Voltage (Volts)",
                () -> voltage.in(Volts),
                null);
        builder.addDoubleProperty(
                "Current (Amps)",
                () -> current.in(Amps),
                null);
    }
}


