package frc.robot.mechanisms.flywheels;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public abstract class Flywheel {

    public final MutableMeasure<Current> current = MutableMeasure.zero(Amps);
    public final MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
    public final MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RadiansPerSecond);

    public Flywheel() {
    }

    public abstract void setInput(double input);

    public abstract double getControlLoopOutput(
            Measure<Velocity<Angle>> currentVelocity,
            Measure<Velocity<Angle>> nextVelocity);

    public abstract void update();
}
