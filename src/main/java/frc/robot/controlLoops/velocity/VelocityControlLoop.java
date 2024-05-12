package frc.robot.controlLoops.velocity;

public interface VelocityControlLoop {

    double getOutput(double currentVelocity, double nextVelocity);


}
