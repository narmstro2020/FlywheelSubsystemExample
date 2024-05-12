package frc.robot.mechanisms.flywheels;

import edu.wpi.first.units.*;
import frc.robot.controlLoops.velocity.VelocityControlLoop;
import frc.robot.motors.Motor;

import static edu.wpi.first.units.Units.Volts;

public class Flywheel {
    public final VelocityControlLoop velocityControlLoop;
    public final Measure<Velocity<Angle>> velocity;
    public final Measure<Current> current;
    public final Measure<Voltage> voltage;
    private final MutableMeasure<Voltage> voltageSetpoint = MutableMeasure.zero(Volts);
    private final Motor motor;


    public Flywheel(
            Motor motor,
            VelocityControlLoop velocityControlLoop) {
        this.current = motor.getCurrent();
        this.voltage = motor.getVoltage();
        this.velocity = motor.getVelocity();
        this.velocityControlLoop = velocityControlLoop;
        this.motor = motor;
    }

    public void setInput(double input) {
        voltageSetpoint.mut_setMagnitude(input);
        motor.setVoltage(voltage);
    }

    public void update() {
        motor.update();
    }
}
