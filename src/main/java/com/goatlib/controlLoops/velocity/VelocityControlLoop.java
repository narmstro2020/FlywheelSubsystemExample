package com.goatlib.controlLoops.velocity;

@FunctionalInterface
public interface VelocityControlLoop {

    double getOutput(double currentVelocity, double nextVelocity);
}
