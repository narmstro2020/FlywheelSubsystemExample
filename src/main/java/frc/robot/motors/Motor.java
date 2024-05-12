package frc.robot.motors;

import edu.wpi.first.units.*;

public interface Motor {

    Measure<Current> getCurrent();

    Measure<Voltage> getVoltage();

    Measure<Angle> getPosition();

    Measure<Velocity<Angle>> getVelocity();

    void setVoltage(Measure<Voltage> voltage);

    void setCurrent(Measure<Current> current);

    void update();

}
