package frc.robot.configurator.rev;

import com.revrobotics.CANSparkBase.IdleMode;

public record REVConfigs(
        int deviceId,
        IdleMode mode,
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
        int periodicStatusFrame6PeriodMs,
        double motorToMechanismConversionFactor
) {
}
