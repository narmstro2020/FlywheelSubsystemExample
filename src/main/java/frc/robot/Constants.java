package frc.robot;

import com.goatlib.configurator.rev.REVEncoderType;
import com.goatlib.configurator.rev.REVMotorType;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.system.plant.DCMotor;
import com.goatlib.configurator.rev.REVConfigs;
import com.goatlib.mechanisms.SimpleMotorConfigs;

public class Constants {

    public static class ExampleFlywheel {
        public static SimpleMotorConfigs flywheelConfigs = new SimpleMotorConfigs(
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
                1.0e-19,
                0.0,
                0.0);

        public static REVConfigs revConfigs = new REVConfigs(
                16,
                REVMotorType.NEOVortexSparkFlex,
                REVEncoderType.EncoderPort,
                CANSparkBase.IdleMode.kCoast,
                false,
                80,
                64,
                100,
                10,
                1,
                20,
                50,
                20,
                200,
                200,
                ExampleFlywheel.flywheelConfigs.gearing()
        );
    }


    private Constants() {
    }
}
