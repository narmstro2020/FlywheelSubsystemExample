package com.goatlib.motorprofiles;

@FunctionalInterface
public interface MotionProfile {

    PositionState calculate(PositionState currentState, PositionState nextState, double dtSeconds);

}
