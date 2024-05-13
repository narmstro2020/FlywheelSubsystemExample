package com.goatlib.controlLoops.position;

import com.goatlib.mechanisms.SimpleMotorConfigs;
import com.goatlib.motorprofiles.ExponentialMotionProfile;
import com.goatlib.motorprofiles.MotionProfile;
import com.goatlib.motorprofiles.PositionState;
import com.goatlib.motorprofiles.TrapezoidMotionProfile;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SimplePIDPositionControlLoop implements PositionControlLoop {

    private final SimpleMotorFeedforward simpleMotorFeedforward;
    private final PIDController pidController;
    private final MotionProfile motionProfile;
    private final double controlLoopPeriodSeconds;

    public static SimplePIDPositionControlLoop createWithTrapezoidProfile(
            SimpleMotorConfigs simpleMotorConfigs,
            double minInput,
            double maxInput) {
        SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(
                simpleMotorConfigs.kS(),
                simpleMotorConfigs.kV(),
                simpleMotorConfigs.kA());
        PIDController pidController = new PIDController(
                simpleMotorConfigs.kP(),
                simpleMotorConfigs.kI(),
                simpleMotorConfigs.kD());
        pidController.enableContinuousInput(minInput, maxInput);
        MotionProfile motionProfile = new TrapezoidMotionProfile(
                simpleMotorFeedforward.maxAchievableVelocity(12.0, 0.0),
                simpleMotorFeedforward.maxAchievableAcceleration(12.0, 0.0));
        return new SimplePIDPositionControlLoop(simpleMotorFeedforward, pidController, motionProfile, simpleMotorConfigs.controlLoopPeriodSeconds());
    }

    public static SimplePIDPositionControlLoop createWithTrapezoidProfile(
            SimpleMotorConfigs simpleMotorConfigs) {
        SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(
                simpleMotorConfigs.kS(),
                simpleMotorConfigs.kV(),
                simpleMotorConfigs.kA());
        PIDController pidController = new PIDController(
                simpleMotorConfigs.kP(),
                simpleMotorConfigs.kI(),
                simpleMotorConfigs.kD());
        MotionProfile motionProfile = new TrapezoidMotionProfile(
                simpleMotorFeedforward.maxAchievableVelocity(12.0, 0.0),
                simpleMotorFeedforward.maxAchievableAcceleration(12.0, 0.0));
        return new SimplePIDPositionControlLoop(simpleMotorFeedforward, pidController, motionProfile, simpleMotorConfigs.controlLoopPeriodSeconds());
    }

    public static SimplePIDPositionControlLoop createwithExponentialProfile(
            SimpleMotorConfigs simpleMotorConfigs,
            double minInput,
            double maxInput) {
        SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(
                simpleMotorConfigs.kS(),
                simpleMotorConfigs.kV(),
                simpleMotorConfigs.kA());
        PIDController pidController = new PIDController(
                simpleMotorConfigs.kP(),
                simpleMotorConfigs.kI(),
                simpleMotorConfigs.kD());
        pidController.enableContinuousInput(minInput, maxInput);
        MotionProfile motionProfile = new ExponentialMotionProfile(simpleMotorConfigs.kV(), simpleMotorConfigs.kA());
        return new SimplePIDPositionControlLoop(simpleMotorFeedforward, pidController, motionProfile, simpleMotorConfigs.controlLoopPeriodSeconds());
    }

    public static SimplePIDPositionControlLoop createwithExponentialProfile(
            SimpleMotorConfigs simpleMotorConfigs) {
        SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(
                simpleMotorConfigs.kS(),
                simpleMotorConfigs.kV(),
                simpleMotorConfigs.kA());
        PIDController pidController = new PIDController(
                simpleMotorConfigs.kP(),
                simpleMotorConfigs.kI(),
                simpleMotorConfigs.kD());
        MotionProfile motionProfile = new ExponentialMotionProfile(simpleMotorConfigs.kV(), simpleMotorConfigs.kA());
        return new SimplePIDPositionControlLoop(simpleMotorFeedforward, pidController, motionProfile, simpleMotorConfigs.controlLoopPeriodSeconds());
    }

    private SimplePIDPositionControlLoop(
            SimpleMotorFeedforward simpleMotorFeedforward,
            PIDController pidController,
            MotionProfile motionProfile,
            double controlLoopPeriodSeconds
    ) {
        this.simpleMotorFeedforward = simpleMotorFeedforward;
        this.pidController = pidController;
        this.motionProfile = motionProfile;
        this.controlLoopPeriodSeconds = controlLoopPeriodSeconds;
    }


    @Override
    public double getOutput(PositionState currentState, PositionState nextState) {
        PositionState actualNextState = motionProfile.calculate(currentState, nextState, controlLoopPeriodSeconds);
        double currentPosition = currentState.position;
        double nextPosition = actualNextState.position;
        double currentVelocity = currentState.velocity;
        double nextVelocity = actualNextState.velocity;
        double voltageFF = simpleMotorFeedforward.calculate(
                currentVelocity,
                nextVelocity,
                controlLoopPeriodSeconds);
        double voltageFB = pidController.calculate(
                currentPosition,
                nextPosition);
        double totalVoltage = voltageFF + voltageFB;
        totalVoltage = MathUtil.clamp(totalVoltage, -12.0, 12.0);
        return totalVoltage;
    }
}
