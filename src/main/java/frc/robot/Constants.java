package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;

public class Constants {

    public enum REVMotorFlywheelConfigs {
        ExampleFlywheel(
                16,
                false,
                80,
                64,
                100,
                10,
                20,
                20,
                50,
                20,
                200,
                200);

        public final int deviceId;
        public final boolean isInverted;
        public final int smartCurrentLimit;
        public final int averageDepth;
        public final int measurementPeriodMs;
        public final int periodicStatusFrame0PeriodMs;
        public final int periodicStatusFrame1PeriodMs;
        public final int periodicStatusFrame2PeriodMs;
        public final int periodicStatusFrame3PeriodMs;
        public final int periodicStatusFrame4PeriodMs;
        public final int periodicStatusFrame5PeriodMs;
        public final int periodicStatusFrame6PeriodMs;

        REVMotorFlywheelConfigs(
                int deviceId,
                boolean isInverted,
                int smartCurrentLimit,
                int averageDepth,
                int measurementPeriodMs,
                int periodicStatusFrame0PeriodMs,
                int periodicStatusFrame1PeriodMs,
                int periodicStatusFrame2PeriodMs,
                int periodicStatusFrame3PeriodMs,
                int periodicStatusFrame4PeriodMs,
                int periodicStatusFrame5PeriodMs,
                int periodicStatusFrame6PeriodMs
        ) {
            this.deviceId = deviceId;
            this.isInverted = isInverted;
            this.smartCurrentLimit = smartCurrentLimit;
            this.averageDepth = averageDepth;
            this.measurementPeriodMs = measurementPeriodMs;
            this.periodicStatusFrame0PeriodMs = periodicStatusFrame0PeriodMs;
            this.periodicStatusFrame1PeriodMs = periodicStatusFrame1PeriodMs;
            this.periodicStatusFrame2PeriodMs = periodicStatusFrame2PeriodMs;
            this.periodicStatusFrame3PeriodMs = periodicStatusFrame3PeriodMs;
            this.periodicStatusFrame4PeriodMs = periodicStatusFrame4PeriodMs;
            this.periodicStatusFrame5PeriodMs = periodicStatusFrame5PeriodMs;
            this.periodicStatusFrame6PeriodMs = periodicStatusFrame6PeriodMs;
        }

    }

    public enum FlywheelConfigs {
        ExampleFlywheel(
                "Example-Flywheel",
                DCMotor.getNeoVortex(1),
                1.0,
                0.01,
                0.00,
                0.01,
                0.00,
                0.0,
                0.017,
                0.001,
                1.0e-19);

        public final String name;
        public final DCMotor gearbox;
        public final double gearing;
        public final double controlLoopPeriodSeconds;
        public final double controlLoopPeriodOffsetSeconds;
        public final double updaterPeriodSeconds;
        public final double updaterPeriodOffsetSeconds;
        public final double kS;
        public final double kV;
        public final double kA;
        public final double kP;

        FlywheelConfigs(
                String name,
                DCMotor gearbox,
                double gearing,
                double controlLoopPeriodSeconds,
                double controlLoopPeriodOffsetSeconds,
                double updaterPeriodSeconds,
                double updaterPeriodOffsetSeconds, double kS, double kV, double kA, double kP) {
            this.name = name;
            this.gearbox = gearbox;
            this.gearing = gearing;
            this.controlLoopPeriodSeconds = controlLoopPeriodSeconds;
            this.controlLoopPeriodOffsetSeconds = controlLoopPeriodOffsetSeconds;
            this.updaterPeriodSeconds = updaterPeriodSeconds;
            this.updaterPeriodOffsetSeconds = updaterPeriodOffsetSeconds;
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kP = kP;
        }

    }


    private Constants() {
    }
}
