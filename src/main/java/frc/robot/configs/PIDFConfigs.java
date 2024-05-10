package frc.robot.configs;

public record PIDFConfigs(
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        double kG) {
}
