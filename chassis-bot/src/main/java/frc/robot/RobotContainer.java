// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveTeleCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.controls.Interlink;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  private final DriveSubsystem driveSubsystem;
  private final Joystick driveJoystick;
  private final Interlink interlink;

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
    driveJoystick = new Joystick(0);
    interlink = new Interlink(driveJoystick);
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(
        new DriveTeleCommand(
            driveSubsystem,
            () -> interlink.getFwd(),
            () -> interlink.getStr(),
            () -> interlink.getYaw()));
    new JoystickButton(driveJoystick, Interlink.InterlinkButton.RESET.id)
        .onTrue(new ResetGyroCommand(driveSubsystem));

    new Trigger(
            () ->
                (driveJoystick.getRawButtonPressed(Interlink.InterlinkButton.HAMBURGER.id)
                    && driveJoystick.getRawButtonPressed(Interlink.InterlinkButton.X.id)))
        .onTrue(new InstantCommand(() -> driveSubsystem.toggleSlow(), driveSubsystem));

    Shuffleboard.getTab("Test").addBoolean("isSlow", () -> driveSubsystem.isSlow());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
