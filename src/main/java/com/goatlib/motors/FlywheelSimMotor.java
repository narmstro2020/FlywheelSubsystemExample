package com.goatlib.motors;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.goatlib.mechanisms.flywheels.FlywheelConfigs;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class FlywheelSimMotor extends DCMotorSim implements Motor {
    private final MutableMeasure<Current> current;
    private final MutableMeasure<Voltage> voltage;
    private final MutableMeasure<Angle> position;
    private final MutableMeasure<Velocity<Angle>> velocity;
    private final double updatePeriodSeconds;

    public FlywheelSimMotor(FlywheelConfigs flywheelConfigs) {
        super(
                LinearSystemId.createDCMotorSystem(
                        flywheelConfigs.kV(),
                        flywheelConfigs.kA()
                ),
                flywheelConfigs.gearbox(),
                flywheelConfigs.gearing());
        current = MutableMeasure.zero(Amps);
        voltage = MutableMeasure.zero(Volts);
        position = MutableMeasure.zero(Radians);
        velocity = MutableMeasure.zero(RadiansPerSecond);
        this.updatePeriodSeconds = flywheelConfigs.updatePeriodSeconds();
    }


    @Override
    public Measure<Current> getCurrent() {
        return null;
    }

    @Override
    public Measure<Voltage> getVoltage() {
        return null;
    }

    @Override
    public Measure<Angle> getPosition() {
        return null;
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return null;
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {

    }

    @Override
    public void setCurrent(Measure<Current> current) {

    }

    @Override
    public void update() {
        current.mut_setMagnitude(getCurrentDrawAmps());
        voltage.mut_setMagnitude(m_u.get(0, 0));
        velocity.mut_setMagnitude(getAngularVelocityRadPerSec());
        position.mut_setMagnitude(getAngularPositionRad());
        update(updatePeriodSeconds);
    }
}
