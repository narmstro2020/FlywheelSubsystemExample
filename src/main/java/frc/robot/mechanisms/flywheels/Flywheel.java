package frc.robot.mechanisms.flywheels;

import edu.wpi.first.units.*;
import frc.robot.controlLoops.velocity.VelocityControlLoop;

import static edu.wpi.first.units.Units.*;

public abstract class Flywheel {

    public final MutableMeasure<Current> current = MutableMeasure.zero(Amps);
    public final MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
    public final MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RadiansPerSecond);
    public final VelocityControlLoop velocityControlLoop;

    public Flywheel(VelocityControlLoop velocityControlLoop) {
        this.velocityControlLoop = velocityControlLoop;
    }

    public abstract void setInput(double input);

    public double getControlLoopOutput(
            Measure<Velocity<Angle>> currentVelocity,
            Measure<Velocity<Angle>> nextVelocity) {
        return velocityControlLoop.getOutput(
                currentVelocity.in(RadiansPerSecond),
                nextVelocity.in(RadiansPerSecond));
    }

    public abstract void update();
}
