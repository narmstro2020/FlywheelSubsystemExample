package com.goatlib.motorprofiles;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TrapezoidMotionProfile implements MotionProfile {

    private final TrapezoidProfile trapezoidProfile;
    private final TrapezoidProfile.State currentState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State nextStateSetpoint = new TrapezoidProfile.State();
    private final PositionState actualNextState = new PositionState();

    public TrapezoidMotionProfile(double maxVelocity, double maxAcceleration) {
        this.trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }

    @Override
    public PositionState calculate(PositionState currentState, PositionState nextStateSetpoint, double dtSeconds) {
        this.currentState.position = currentState.position;
        this.currentState.velocity = currentState.velocity;
        this.nextStateSetpoint.position = nextStateSetpoint.position;
        this.nextStateSetpoint.velocity = nextStateSetpoint.velocity;
        TrapezoidProfile.State actualNextState = trapezoidProfile.calculate(dtSeconds, this.currentState, this.nextStateSetpoint);
        this.actualNextState.position = actualNextState.position;
        this.actualNextState.velocity = actualNextState.velocity;
        return this.actualNextState;

    }
}
