package frc.robot.subsystems;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SimFlywheel extends FlywheelSim {
    public SimFlywheel(LinearSystem<N1, N1, N1> plant, DCMotor gearbox, double gearing) {
        super(plant, gearbox, gearing);
    }

    public double getInputVoltage(){
        return m_u.get(0,0);
    }
}
