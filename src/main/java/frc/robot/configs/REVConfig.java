package frc.robot.configs;

import com.revrobotics.CANSparkFlex;

import static com.revrobotics.CANSparkLowLevel.*;

public record REVConfig(
        int deviceId) {

    public static CANSparkFlex createSparkFlex(
            MotorType type,
            REVConfig revConfig,
            double conversionFactor) {
        //TODO:  add more configuration options later
        return new CANSparkFlex(revConfig.deviceId(), type);
    }
}


