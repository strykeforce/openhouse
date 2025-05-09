// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.Swerve;
import frc.robot.Interlink;
import frc.robot.ResetGyroCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem;
  private final Interlink interlink;
  private final Swerve swerve;
  private final Joystick joystick;
  public RobotContainer() {
    swerve = new Swerve();
    driveSubsystem = new DriveSubsystem(swerve);
    joystick = new Joystick(0);
    interlink = new Interlink(joystick);
    configureBindings();
  }

  private void configureBindings() {driveSubsystem.setDefaultCommand(
    new DriveTeleopCommand(
        () -> interlink.getFwd(),
        () -> interlink.getStr(),
        () -> interlink.getYaw(),
        driveSubsystem));
        new JoystickButton(joystick, Interlink.InterlinkButton.RESET.id)
        .onTrue(new ResetGyroCommand(driveSubsystem));
        new Trigger(
          () ->
                (joystick.getRawButtonPressed(Interlink.Trim.LEFT_X_NEG.id)
                    && joystick.getRawButtonPressed(Interlink.Trim.RIGHT_X_POS.id)
                    && joystick.getRawButtonPressed(Interlink.InterlinkButton.UP.id)))
                    .onTrue(new boringDriving(driveSubsystem));
        }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
