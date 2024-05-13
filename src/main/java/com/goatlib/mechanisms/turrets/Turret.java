package com.goatlib.mechanisms.turrets;

import com.goatlib.controlLoops.position.PositionControlLoop;
import edu.wpi.first.units.*;
import com.goatlib.motors.Motor;

import static edu.wpi.first.units.Units.Volts;

public class Turret {
    public final PositionControlLoop positionControlLoop;
    public final Measure<Angle> position;
    public final Measure<Velocity<Angle>> velocity;
    public final Measure<Current> current;
    public final Measure<Voltage> voltage;
    private final MutableMeasure<Voltage> voltageSetpoint = MutableMeasure.zero(Volts);
    private final Motor motor;


    public Turret(
            Motor motor,
            PositionControlLoop positionControlLoop, Measure<Angle> position) {
        this.position = position;
        this.velocity = motor.getVelocity();
        this.current = motor.getCurrent();
        this.voltage = motor.getVoltage();
        this.positionControlLoop = positionControlLoop;
        this.motor = motor;
    }

    public void setInput(double input) {
        voltageSetpoint.mut_setMagnitude(input);
        motor.setVoltage(voltage);
    }

    public void update() {
        motor.update();
    }
}
