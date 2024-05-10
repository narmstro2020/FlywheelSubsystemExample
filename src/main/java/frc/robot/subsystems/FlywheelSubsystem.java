package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

public class FlywheelSubsystem extends SubsystemBase {

    public static FlywheelSubsystem createSimulatedPIDF(
            String name,
            double kV,
            double kA,
            double kP,
            DCMotor gearbox,
            double gearing,
            double controlLoopPeriodSeconds,
            double updatePeriodSeconds,
            BiConsumer<Runnable, Double> addPeriodicMethod) {

        MutableMeasure<Current> current = MutableMeasure.zero(Amps);
        MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
        MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RadiansPerSecond);
        LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(kV, kA);
        SimFlywheel simFlywheel = new SimFlywheel(plant, gearbox, gearing);
        SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(0, kV, kA);
        PIDController pidController = new PIDController(kP, 0, 0, controlLoopPeriodSeconds);
        MutableMeasure<Voltage> controlLoopOutput = MutableMeasure.zero(Volts);

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
                (currentVelocity, nextVelocity) -> {
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
                },
                controlLoopPeriodSeconds,
                updatePeriodSeconds,
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
            double updatePeriodSeconds,
            BiConsumer<Runnable, Double> addPeriodicMethod) {
        this.current = current;
        this.voltage = voltage;
        this.velocity = velocity;

        addPeriodicMethod.accept(
                updater,
                updatePeriodSeconds
        );

        addPeriodicMethod.accept(
                () -> {
                    if (!sysIdActive) {
                        voltageConsumer.accept(
                                controlLoop.apply(velocity, velocitySetpoint)
                        );
                    }
                },
                controlLoopPeriodSeconds);

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

        setDefaultCommand(
                Commands.sequence(
                                runOnce(() -> sysIdActive = false),
                                run(() -> velocitySetpoint.mut_setMagnitude(velocity.in(RadiansPerSecond))))
                        .withName("STOP"));
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


