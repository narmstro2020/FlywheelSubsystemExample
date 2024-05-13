// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.goatlib.controlLoops.position.SimplePIDPositionControlLoop;
import com.goatlib.mechanisms.turrets.Turret;
import com.goatlib.motors.rev.REVRelativeMotor;
import com.goatlib.periodic.PeriodicTask;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.goatlib.controlLoops.velocity.SimplePIDFVelocityControlLoop;
import com.goatlib.mechanisms.flywheels.Flywheel;
import com.goatlib.motors.SimMotor;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;


public class RobotContainer {

    private final CommandXboxController commandXboxController = new CommandXboxController(0);
    private final FlywheelSubsystem exampleFlywheelSubsystem;
    private final TurretSubsystem exampleTurretSubsystem;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer(PeriodicTask addPeriodic) {
        exampleFlywheelSubsystem = new FlywheelSubsystem(
                new Flywheel(
                        RobotBase.isReal()
                                ? new REVRelativeMotor(Constants.ExampleFlywheel.revConfigs)
                                : new SimMotor(Constants.ExampleFlywheel.flywheelConfigs),
                        new SimplePIDFVelocityControlLoop(Constants.ExampleFlywheel.flywheelConfigs)),
                Constants.ExampleFlywheel.flywheelConfigs,
                addPeriodic);
        exampleTurretSubsystem = new TurretSubsystem(
                new Turret(
                        RobotBase.isReal()
                                ? new REVRelativeMotor(Constants.ExampleTurret.revConfigs)
                                : new SimMotor(Constants.ExampleTurret.simpleMotorConfigs),
                        SimplePIDPositionControlLoop.createWithTrapezoidProfile(
                                Constants.ExampleTurret.simpleMotorConfigs,
                                -Math.PI / 2,
                                Math.PI / 2)),
                Constants.ExampleFlywheel.flywheelConfigs,
                addPeriodic);
        SmartDashboard.putData("Example Flywheel", exampleFlywheelSubsystem);
        SmartDashboard.putData("Example Turret", exampleTurretSubsystem);
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
        commandXboxController.b().whileTrue(
                exampleTurretSubsystem.createSetPositionCommand(Degrees.of(45)));
        commandXboxController.b().onFalse(
                exampleTurretSubsystem.createHoldCommand());
        commandXboxController.x().whileTrue(
                exampleTurretSubsystem.createSetPositionCommand(Degrees.of(0.0)));
        commandXboxController.x().onFalse(
                exampleTurretSubsystem.createHoldCommand());
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
