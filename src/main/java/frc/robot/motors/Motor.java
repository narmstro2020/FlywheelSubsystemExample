package frc.robot.motors;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public interface Motor {

    public Measure<Current> getCurrent();

    public Measure<Voltage> getVoltage();

    public Measure<Angle> getPosition();

    public Measure<Velocity<Angle>> getVelocity();

    public void setVoltage(Measure<Voltage> voltage);

    public void setCurrent(Measure<Current> current);

    public void update();

}
