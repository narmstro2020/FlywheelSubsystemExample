package frc.robot.mechanisms.flywheels;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.controlLoops.velocity.VelocityControlLoop;
import frc.robot.subsystems.SimFlywheel;

public class FlywheelSimulated extends Flywheel {

    private final SimFlywheel simFlywheel;
    private final double updatePeriodSeconds;

    public FlywheelSimulated(FlywheelConfigs flywheelConfigs, VelocityControlLoop velocityControlLoop) {
        super(velocityControlLoop);

        LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(
                flywheelConfigs.kV(),
                flywheelConfigs.kA());
        simFlywheel = new SimFlywheel(
                plant,
                flywheelConfigs.gearbox(),
                flywheelConfigs.gearing());
        updatePeriodSeconds = flywheelConfigs.updatePeriodSeconds();
    }

    @Override
    public void setInput(double input) {
        simFlywheel.setInputVoltage(input);
    }

    @Override
    public void update() {
        current.mut_setMagnitude(simFlywheel.getCurrentDrawAmps());
        voltage.mut_setMagnitude(simFlywheel.getInputVoltage());
        velocity.mut_setMagnitude(simFlywheel.getAngularVelocityRadPerSec());
        simFlywheel.update(updatePeriodSeconds);
    }
}
