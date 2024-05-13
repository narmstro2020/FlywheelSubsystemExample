package com.goatlib.controlLoops.position;

import com.goatlib.motorprofiles.PositionState;

@FunctionalInterface
public interface PositionControlLoop {
    double getOutput(PositionState currentState, PositionState nextState);
}
