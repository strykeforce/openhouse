package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.drive.DriveSubsystem;

public class boringDriving extends InstantCommand {
  DriveSubsystem driveSubsystem;

  public boringDriving(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.toggleSafeDriving();
  }
}
