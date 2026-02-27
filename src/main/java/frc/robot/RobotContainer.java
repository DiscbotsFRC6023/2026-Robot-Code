// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);

  /* Subsystems */
  private final Swerve swerve = new Swerve();

  public RobotContainer() {
    /* Set default commands */
    swerve.setDefaultCommand(
        SubsystemCommands.teleopDrive(
            swerve,
            () -> -driver.getLeftY(),  // forward/back (negate because stick Y is inverted)
            () -> -driver.getLeftX(),  // strafe left/right
            () -> -driver.getRightX() // rotation
        )
    );

    configureBindings();
  }

  private void configureBindings() {
    /* Back button → zero gyro heading */
    driver.back().onTrue(SubsystemCommands.zeroGyro(swerve));

    /* Start button → zero gyro heading (alternative) */
    driver.start().onTrue(SubsystemCommands.zeroGyro(swerve));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /* Expose subsystems if needed */
  public Swerve getSwerve() {
    return swerve;
  }
}
