package frc.robot.controlLoops.velocity;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class SimpleLQRVelocityControlLoop implements VelocityControlLoop {

    private final Vector<N1> currentReference;
    private final Vector<N1> nextReference;
    private final LinearSystemLoop<N1, N1, N1> loop;
    private final double controlLoopPeriodSeconds;

    public SimpleLQRVelocityControlLoop(
            double kV,
            double kA,
            double stateStdDev,
            double measurementStdDev,
            double velocityErrorTolerance,
            double maxControlEffortVolts,
            double controlLoopPeriodSeconds) {
        LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(kV, kA);
        KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
                Nat.N1(),
                Nat.N1(),
                plant,
                MatBuilder.fill(Nat.N1(), Nat.N1(), stateStdDev),
                MatBuilder.fill(Nat.N1(), Nat.N1(), measurementStdDev),
                controlLoopPeriodSeconds);
        LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(
                plant,
                VecBuilder.fill(velocityErrorTolerance),
                VecBuilder.fill(maxControlEffortVolts),
                controlLoopPeriodSeconds);
        loop = new LinearSystemLoop<>(
                plant,
                controller,
                observer,
                maxControlEffortVolts,
                controlLoopPeriodSeconds);
        this.controlLoopPeriodSeconds = controlLoopPeriodSeconds;

        currentReference = VecBuilder.fill(0.0);
        nextReference = VecBuilder.fill(0.0);
    }

    @Override
    public double getOutput(double currentVelocity, double nextVelocity) {
        currentReference.set(0, 0, currentVelocity);
        nextReference.set(0, 0, nextVelocity);
        loop.setNextR(nextReference);
        loop.correct(currentReference);
        loop.predict(controlLoopPeriodSeconds);
        return loop.getU(0);
    }
}

