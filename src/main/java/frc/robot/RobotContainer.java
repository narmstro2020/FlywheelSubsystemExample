// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.goatlib.AddPeriodic;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.goatlib.controlLoops.velocity.SimplePIDFVelocityControlLoop;
import com.goatlib.mechanisms.flywheels.Flywheel;
import com.goatlib.motors.FlywheelSimMotor;
import com.goatlib.motors.NEOVortexMotor;
import frc.robot.subsystems.FlywheelSubsystem;

import static edu.wpi.first.units.Units.RPM;


public class RobotContainer {

    private final CommandXboxController commandXboxController = new CommandXboxController(0);
    private final FlywheelSubsystem exampleFlywheelSubsystem;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer(AddPeriodic addPeriodic) {
        exampleFlywheelSubsystem = RobotBase.isReal()
                ?
                new FlywheelSubsystem(
                        new Flywheel(
                                new NEOVortexMotor(Constants.ExampleFlywheel.revConfigs),
                                new SimplePIDFVelocityControlLoop(Constants.ExampleFlywheel.flywheelConfigs)),
                        Constants.ExampleFlywheel.flywheelConfigs,
                        addPeriodic)
                :
                new FlywheelSubsystem(
                        new Flywheel(
                                new FlywheelSimMotor(Constants.ExampleFlywheel.flywheelConfigs),
                                new SimplePIDFVelocityControlLoop(Constants.ExampleFlywheel.flywheelConfigs)),
                        Constants.ExampleFlywheel.flywheelConfigs,
                        addPeriodic);
        SmartDashboard.putData("Example Flywheel", exampleFlywheelSubsystem);
        autoChooser.addOption("NONE", Commands.none());
        autoChooser.addOption("ExampleFlywheelSysIdQuasiForward", exampleFlywheelSubsystem.sysIdQuasistaticForward());
        autoChooser.addOption("ExampleFlywheelSysIdQuasiReverse", exampleFlywheelSubsystem.sysIdQuasistaticReverse());
        autoChooser.addOption("ExampleFlywheelSysIdDynamicForward", exampleFlywheelSubsystem.sysIdDynamicForward());
        autoChooser.addOption("ExampleFlywheelSysIdDynamicReverse", exampleFlywheelSubsystem.sysIdDynamicReverse());
        configureBindings();
    }


    private void configureBindings() {
        commandXboxController.a().whileTrue(
                exampleFlywheelSubsystem.createSetVelocityCommand(RPM.of(3000)));
        commandXboxController.a().onFalse(
                exampleFlywheelSubsystem.createSetVelocityCommand(RPM.of(0.0)).withName("STOP"));
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
