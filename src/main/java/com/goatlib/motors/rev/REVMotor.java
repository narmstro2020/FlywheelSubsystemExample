package com.goatlib.motors.rev;

import com.goatlib.configurator.rev.REVMotorType;
import com.goatlib.motors.Motor;
import com.revrobotics.*;
import edu.wpi.first.units.*;
import com.goatlib.configurator.rev.REVConfigs;
import com.goatlib.configurator.rev.REVConfigurator;
import static edu.wpi.first.units.Units.*;

public abstract class REVMotor implements Motor {

    protected final MutableMeasure<Current> current;
    protected final MutableMeasure<Voltage> voltage;
    protected final MutableMeasure<Angle> position;
    protected final MutableMeasure<Velocity<Angle>> velocity;
    protected final CANSparkBase canSparkBase;
    protected final REVConfigurator configurator;


    public REVMotor(REVConfigs revConfigs) {
        current = MutableMeasure.zero(Amps);
        voltage = MutableMeasure.zero(Volts);
        position = MutableMeasure.zero(Radians);
        velocity = MutableMeasure.zero(RadiansPerSecond);

        canSparkBase = revConfigs.revMotorType() == REVMotorType.NEOVortexSparkFlex
                ? new CANSparkFlex(revConfigs.deviceId(), CANSparkLowLevel.MotorType.kBrushless)
                : new CANSparkMax(revConfigs.deviceId(), CANSparkLowLevel.MotorType.kBrushless);

        configurator = REVConfigurator.configure(canSparkBase)
                .withIdleMode(revConfigs.mode())
                .withInverted(revConfigs.isInverted())
                .withSmartCurrentLimit(revConfigs.smartCurrentLimit())
                .withPeriodicStatusFrame0Period(revConfigs.periodicStatusFrame1PeriodMs())
                .withPeriodicStatusFrame1Period(revConfigs.periodicStatusFrame2PeriodMs())
                .withPeriodicStatusFrame2Period(revConfigs.periodicStatusFrame2PeriodMs())
                .withPeriodicStatusFrame3Period(revConfigs.periodicStatusFrame3PeriodMs())
                .withPeriodicStatusFrame4Period(revConfigs.periodicStatusFrame4PeriodMs())
                .withPeriodicStatusFrame5Period(revConfigs.periodicStatusFrame5PeriodMs())
                .withPeriodicStatusFrame6Period(revConfigs.periodicStatusFrame6PeriodMs());
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
        canSparkBase.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setCurrent(Measure<Current> current) {
        canSparkBase.getPIDController().setReference(current.in(Amps), CANSparkBase.ControlType.kCurrent);
    }

    public void update() {
        current.mut_setMagnitude(canSparkBase.getOutputCurrent());
        voltage.mut_setMagnitude(canSparkBase.getAppliedOutput() * canSparkBase.getBusVoltage());
    }


}
