package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.configs.PIDFConfigs;
import frc.robot.configs.REVConfig;

class Constants {

    static class ExampleFlywheel {
        static String name = "Example-Flywheel";

        static REVConfig revConfig = new REVConfig(
                26
        );
        static PIDFConfigs pidConfigs = new PIDFConfigs(
                1.0e-19,
                0.0,
                0.0,
                0.0,
                0.017,
                0.001,
                0.0);
        static DCMotor gearbox = DCMotor.getNeoVortex(1);
        static double gearing = 1.0;
        static double controlLoopPeriodSeconds = 0.01;
        static double controlLoopPeriodOffsetSeconds = 0.00;
        static double updaterPeriodSeconds = 0.01;
        static double updaterPeriodOffsetSeconds = 0.00;
    }


    private Constants() {
    }
}
