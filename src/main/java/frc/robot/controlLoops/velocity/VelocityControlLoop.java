package frc.robot.controlLoops.velocity;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;


import java.util.function.BiFunction;

import static edu.wpi.first.units.Units.*;

public interface VelocityControlLoop {

    double getOutput(Measure<Velocity<Angle>> currentVelocity, Measure<Velocity<Angle>> nextVelocity);

    static BiFunction<Measure<Velocity<Angle>>, Measure<Velocity<Angle>>, Measure<Voltage>> createFlywheelLQR(
            double kV,
            double kA,
            double stateStdDev,
            double measurementStdDev,
            double velocityErrorTolerance,
            double maxControlEffortVolts,
            double controlLoopPeriodSeconds
    ) {
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
        LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<>(
                plant,
                controller,
                observer,
                maxControlEffortVolts,
                controlLoopPeriodSeconds);

        MutableMeasure<Voltage> controlLoopOutput = MutableMeasure.zero(Volts);

        Vector<N1> currentReference = VecBuilder.fill(0.0);
        Vector<N1> nextReference = VecBuilder.fill(0.0);

        return (currentVelocity, nextVelocity) -> {
            currentReference.set(0, 0, currentVelocity.in(RadiansPerSecond));
            nextReference.set(0, 0, nextVelocity.in(RadiansPerSecond));
            loop.setNextR(nextReference);
            loop.correct(currentReference);
            loop.predict(controlLoopPeriodSeconds);
            controlLoopOutput.mut_setMagnitude(loop.getU(0));
            return controlLoopOutput;
        };
    }

}