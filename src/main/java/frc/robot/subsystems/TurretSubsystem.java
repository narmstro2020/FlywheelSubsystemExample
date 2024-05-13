package frc.robot.subsystems;


import com.goatlib.mechanisms.SimpleMotorConfigs;
import com.goatlib.mechanisms.turrets.Turret;
import com.goatlib.motorprofiles.PositionState;
import com.goatlib.periodic.PeriodicTask;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

public class TurretSubsystem extends SubsystemBase {

    private final Turret turret;
    private final PositionState currentPositionState;
    private final PositionState nextPositionState;
    private final MutableMeasure<Angle> positionSetpoint = MutableMeasure.zero(Radians);
    private final SysIdRoutine sysIdRoutine;
    private boolean sysIdActive = false;


    public TurretSubsystem(
            Turret turret,
            SimpleMotorConfigs turretConfigs,
            PeriodicTask addPeriodic) {
        this.turret = turret;
        this.currentPositionState = new PositionState();
        this.nextPositionState = new PositionState();

        addPeriodic.accept(turret::update, turretConfigs.updatePeriodSeconds(), turretConfigs.updatePeriodOffsetSeconds());

        addPeriodic.accept(
                () -> {
                    if (!sysIdActive) {
                        currentPositionState.position = turret.position.in(Radians);
                        currentPositionState.velocity = turret.velocity.in(RadiansPerSecond);
                        nextPositionState.position = positionSetpoint.in(Radians);
                        nextPositionState.velocity = 0.0;
                        turret.setInput(
                                turret.positionControlLoop.getOutput(
                                        currentPositionState,
                                        nextPositionState));
                    }
                },
                turretConfigs.controlLoopPeriodSeconds(),
                turretConfigs.controlLoopPeriodOffsetSeconds());

        sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motor(s).
                        (voltage) -> turret.setInput(voltage.in(Volts)),
                        // Tell SysId how to record a frame of data for each motor on the mechanism being
                        // characterized.
                        log -> {
                            // Record a frame for the shooter motor.
                            log.motor("turret")
                                    .voltage(turret.voltage)
                                    .angularPosition(turret.position)
                                    .angularVelocity(turret.velocity);
                        },
                        // Tell SysId to make generated commands require this subsystem, suffix test state in
                        // WPILog with this subsystem's name ("shooter")
                        this,
                        turretConfigs.name()));
    }

    public Trigger createAtSetpointTrigger(Measure<Angle> setpoint, Measure<Angle> tolerance) {
        return new Trigger(() -> MathUtil.isNear(setpoint.in(Radians), turret.position.in(Radians), tolerance.in(Radians)))
                .and(() -> MathUtil.isNear(0.0, turret.velocity.in(RadiansPerSecond), 0.0));
    }

    public Command createHoldCommand(){
        return Commands.sequence(
                        runOnce(() -> sysIdActive = false),
                        run(() -> positionSetpoint.mut_setMagnitude(positionSetpoint.in(Radians))))
                .withName(String.format("hold position at %s degrees", positionSetpoint.in(Degrees)));
    }

    public Command createSetPositionCommand(Measure<Angle> position) {
        return Commands.sequence(
                        runOnce(() -> sysIdActive = false),
                        run(() -> positionSetpoint.mut_setMagnitude(position.in(Radians))))
                .withName(String.format("position set to %s degrees", position.in(Degrees)));
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
                "Position (degrees)",
                () -> turret.position.in(Degrees),
                null);
        builder.addDoubleProperty(
                "Position Setpoint (degrees)",
                () -> positionSetpoint.in(Degrees),
                null);
        builder.addDoubleProperty(
                "Voltage (Volts)",
                () -> turret.voltage.in(Volts),
                null);
        builder.addDoubleProperty(
                "Current (Amps)",
                () -> turret.current.in(Amps),
                null);
    }
}


