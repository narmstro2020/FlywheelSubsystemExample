package frc.robot.controlLoops.velocity;

import edu.wpi.first.units.*;

public interface VelocityControlLoop {

    double getOutput(Measure<Velocity<Angle>> currentVelocity, Measure<Velocity<Angle>> nextVelocity);


}
