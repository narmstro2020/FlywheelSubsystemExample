package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.configs.PIDFConfigs;
import frc.robot.configs.REVConfig;

import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

import static com.revrobotics.CANSparkLowLevel.*;
import static edu.wpi.first.units.Units.*;

public class FlywheelSubsystem extends SubsystemBase {

    public static FlywheelSubsystem createNEOVortexPIDF(
            String name,
            REVConfig revConfig,
            BiFunction<Measure<Velocity<Angle>>, Measure<Velocity<Angle>>, Measure<Voltage>> controlLoop,
            double gearing,
            double controlLoopPeriodSeconds,
            double controlLoopPeriodOffsetSeconds,
            double updatePeriodSeconds,
            double updatePeriodOffsetSeconds,
            Function<Runnable, BiConsumer<Double, Double>> addPeriodicMethod) {
        var canSparkFlex = REVConfig.createSparkFlex(MotorType.kBrushless, revConfig, gearing);
        MutableMeasure<Current> current = MutableMeasure.zero(Amps);
        MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
        MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RadiansPerSecond);
        return new FlywheelSubsystem(
                name,
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
                controlLoopPeriodSeconds,
                controlLoopPeriodOffsetSeconds,
                updatePeriodSeconds,
                updatePeriodOffsetSeconds,
                addPeriodicMethod

        );
    }

    public static FlywheelSubsystem createSimulated(
            String name,
            PIDFConfigs pidfConfigs,
            DCMotor gearbox,
            double gearing,
            BiFunction<Measure<Velocity<Angle>>, Measure<Velocity<Angle>>, Measure<Voltage>> controlLoop,
            double controlLoopPeriodSeconds,
            double controlLoopPeriodOffsetSeconds,
            double updatePeriodSeconds,
            double updatePeriodOffsetSeconds,
            Function<Runnable, BiConsumer<Double, Double>> addPeriodicMethod) {

        MutableMeasure<Current> current = MutableMeasure.zero(Amps);
        MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
        MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RadiansPerSecond);
        LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(
                pidfConfigs.kV(),
                pidfConfigs.kA());
        SimFlywheel simFlywheel = new SimFlywheel(plant, gearbox, gearing);

        return new FlywheelSubsystem(
                name,
                current,
                voltage,
                velocity,
                (inputVoltage) -> simFlywheel.setInputVoltage(inputVoltage.in(Volts)),
                () -> {
                    current.mut_setMagnitude(simFlywheel.getCurrentDrawAmps());
                    voltage.mut_setMagnitude(simFlywheel.getInputVoltage());
                    velocity.mut_setMagnitude(simFlywheel.getAngularVelocityRadPerSec());
                    simFlywheel.update(updatePeriodSeconds);
                },
                controlLoop,
                controlLoopPeriodSeconds,
                controlLoopPeriodOffsetSeconds,
                updatePeriodSeconds,
                updatePeriodOffsetSeconds,
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


