// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.controlLoops.VelocityControlLoop;
import frc.robot.subsystems.FlywheelSubsystem;

import java.util.function.BiConsumer;
import java.util.function.Function;

import static edu.wpi.first.units.Units.RPM;


public class RobotContainer {

    private final CommandXboxController commandXboxController = new CommandXboxController(0);
    private final FlywheelSubsystem exampleFlywheelSubsystem;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer(Function<Runnable, BiConsumer<Double, Double>> addPeriodicMethod) {
        exampleFlywheelSubsystem = RobotBase.isReal()
                ?
                FlywheelSubsystem.createNEOVortexPIDF(
                        Constants.ExampleFlywheel.name,
                        Constants.ExampleFlywheel.revConfig,
                        VelocityControlLoop.createFlywheelPIDF(
                                Constants.ExampleFlywheel.pidConfigs,
                                Constants.ExampleFlywheel.controlLoopPeriodSeconds),
                        Constants.ExampleFlywheel.gearing,
                        Constants.ExampleFlywheel.controlLoopPeriodSeconds,
                        Constants.ExampleFlywheel.controlLoopPeriodOffsetSeconds,
                        Constants.ExampleFlywheel.updaterPeriodSeconds,
                        Constants.ExampleFlywheel.updaterPeriodOffsetSeconds,
                        addPeriodicMethod)
                :
                FlywheelSubsystem.createSimulated(
                        Constants.ExampleFlywheel.name,
                        Constants.ExampleFlywheel.pidConfigs,
                        Constants.ExampleFlywheel.gearbox,
                        Constants.ExampleFlywheel.gearing,
                        VelocityControlLoop.createFlywheelPIDF(
                                Constants.ExampleFlywheel.pidConfigs,
                                Constants.ExampleFlywheel.controlLoopPeriodSeconds),
                        Constants.ExampleFlywheel.controlLoopPeriodSeconds,
                        Constants.ExampleFlywheel.controlLoopPeriodOffsetSeconds,
                        Constants.ExampleFlywheel.updaterPeriodSeconds,
                        Constants.ExampleFlywheel.updaterPeriodOffsetSeconds,
                        addPeriodicMethod);
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
