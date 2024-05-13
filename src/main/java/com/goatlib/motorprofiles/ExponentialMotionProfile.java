package com.goatlib.motorprofiles;

import edu.wpi.first.math.trajectory.ExponentialProfile;

public class ExponentialMotionProfile implements MotionProfile {

    private final ExponentialProfile exponentialProfile;
    private final ExponentialProfile.State currentState = new ExponentialProfile.State();
    private final ExponentialProfile.State nextStateSetpoint = new ExponentialProfile.State();
    private final PositionState actualNextState = new PositionState();

    public ExponentialMotionProfile(double kV, double kA) {
        this.exponentialProfile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(12.0, kV, kA));
    }

    @Override
    public PositionState calculate(PositionState currentState, PositionState nextStateSetpoint, double dtSeconds) {
        this.currentState.position = currentState.position;
        this.currentState.velocity = currentState.velocity;
        this.nextStateSetpoint.position = nextStateSetpoint.position;
        this.nextStateSetpoint.velocity = nextStateSetpoint.velocity;
        ExponentialProfile.State actualNextState = exponentialProfile.calculate(dtSeconds, this.currentState, this.nextStateSetpoint);
        this.actualNextState.position = actualNextState.position;
        this.actualNextState.velocity = actualNextState.velocity;
        return this.actualNextState;

    }
}
