package frc.robot.controlLoops.velocity;

import edu.wpi.first.units.*;

public interface VelocityControlLoop {

    double getOutput(double currentVelocity, double nextVelocity);


}
