package com.goatlib.motors;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.units.*;
import com.goatlib.configurator.rev.REVConfigs;
import com.goatlib.configurator.rev.REVConfigurator;

import static edu.wpi.first.units.Units.*;

public class NEOVortexMotor implements Motor {

    private final MutableMeasure<Current> current;
    private final MutableMeasure<Voltage> voltage;
    private final MutableMeasure<Angle> position;
    private final MutableMeasure<Velocity<Angle>> velocity;
    private final CANSparkFlex canSparkFlex;


    public NEOVortexMotor(REVConfigs revConfigs) {
        current = MutableMeasure.zero(Amps);
        voltage = MutableMeasure.zero(Volts);
        position = MutableMeasure.zero(Radians);
        velocity = MutableMeasure.zero(RadiansPerSecond);
        canSparkFlex = new CANSparkFlex(revConfigs.deviceId(), CANSparkLowLevel.MotorType.kBrushless);
        REVConfigurator configurator = REVConfigurator.configure(canSparkFlex)
                .withIdleMode(revConfigs.mode())
                .withInverted(revConfigs.isInverted())
                .withAverageDepth(canSparkFlex.getEncoder(), revConfigs.averageDepth())
                .withMeasurementPeriod(canSparkFlex.getEncoder(), revConfigs.measurementPeriodMs())
                .withSmartCurrentLimit(revConfigs.smartCurrentLimit())
                .withPeriodicStatusFrame0Period(revConfigs.periodicStatusFrame1PeriodMs())
                .withPeriodicStatusFrame1Period(revConfigs.periodicStatusFrame2PeriodMs())
                .withPeriodicStatusFrame2Period(revConfigs.periodicStatusFrame2PeriodMs())
                .withPeriodicStatusFrame3Period(revConfigs.periodicStatusFrame3PeriodMs())
                .withPeriodicStatusFrame4Period(revConfigs.periodicStatusFrame4PeriodMs())
                .withPeriodicStatusFrame5Period(revConfigs.periodicStatusFrame5PeriodMs())
                .withPeriodicStatusFrame6Period(revConfigs.periodicStatusFrame6PeriodMs())
                .withConversionFactor(canSparkFlex.getEncoder(), revConfigs.motorToMechanismConversionFactor());
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
        canSparkFlex.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setCurrent(Measure<Current> current) {
        canSparkFlex.getPIDController().setReference(current.in(Amps), CANSparkBase.ControlType.kCurrent);
    }

    @Override
    public void update() {
        current.mut_setMagnitude(canSparkFlex.getOutputCurrent());
        voltage.mut_setMagnitude(canSparkFlex.getAppliedOutput() * canSparkFlex.getBusVoltage());
        position.mut_setMagnitude(canSparkFlex.getEncoder().getPosition());
        velocity.mut_setMagnitude(canSparkFlex.getEncoder().getVelocity());
    }
}
