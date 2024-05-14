package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class DriveTeleCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private DoubleSupplier fwdStick;
  private DoubleSupplier strStick;
  private DoubleSupplier yawStick;
  private double fwdStrScale;
  private double yawScale;
  private double fwd;
  private double str;
  private double yaw;

  public DriveTeleCommand(
      DriveSubsystem driveSubsystem,
      DoubleSupplier fwdStick,
      DoubleSupplier strStick,
      DoubleSupplier yawStick) {
    this.driveSubsystem = driveSubsystem;
    this.fwdStick = fwdStick;
    this.strStick = strStick;
    this.yawStick = yawStick;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    fwdStrScale = driveSubsystem.getFwdStrScale();
    yawScale = driveSubsystem.getYawScale();

    fwd =
        MathUtil.applyDeadband(fwdStick.getAsDouble(), Constants.kDeadbandAllStick)
            * fwdStrScale
            * Constants.kMaxSpeedMetersPerSecond;
    str =
        MathUtil.applyDeadband(strStick.getAsDouble(), Constants.kDeadbandAllStick)
            * fwdStrScale
            * Constants.kMaxSpeedMetersPerSecond;
    yaw =
        MathUtil.applyDeadband(yawStick.getAsDouble(), Constants.kDeadbandAllStick)
            * yawScale
            * Constants.kMaxOmega;

    driveSubsystem.drive(-fwd, -str, -yaw, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true);
  }
}
