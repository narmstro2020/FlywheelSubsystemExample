package com.goatlib.motors;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.goatlib.mechanisms.SimpleMotorConfigs;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class SimMotor extends DCMotorSim implements Motor {
    private final MutableMeasure<Current> current;
    private final MutableMeasure<Voltage> voltage;
    private final MutableMeasure<Angle> position;
    private final MutableMeasure<Velocity<Angle>> velocity;
    private final double updatePeriodSeconds;

    public SimMotor(SimpleMotorConfigs flywheelConfigs) {
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
        return current;
    }

    @Override
    public Measure<Voltage> getVoltage() {
        return voltage;
    }

    @Override
    public Measure<Angle> getPosition() {
        return position;
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return velocity;
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {
        setInputVoltage(voltage.in(Volts));
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
