// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LedSubsystem.LedState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private Servo pointer;

  private TalonSRX turret;
  private DeadEyeSubsystem deadeye;
  private LedSubsystem ledSubsystem;
  private wallEYESubsystem wallEYE;

  private Boolean isOnColor = false;

  @Override
  public void robotInit() {
    pointer = new Servo(Constants.kServoID);
    turret = new TalonSRX(Constants.kFalconSRXID);
    deadeye = new DeadEyeSubsystem();
    ledSubsystem = new LedSubsystem();
    wallEYE = new wallEYESubsystem();

    turret.configFactoryDefault();
    turret.configAllSettings(Constants.getSrxConfiguration());
    turret.setSelectedSensorPosition(0.0);
    ledSubsystem.setFlaming();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // System.out.println(deadeye.getDistanceToCamCenter());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  public double clamp(double min, double max, double n) {
    assert max > min;
    if (n > max) {
      return max;
    }
    if (n < min) {
      return min;
    }
    return n;
  }

  @Override
  public void teleopPeriodic() {

    double center = deadeye.getDistanceToCamCenter();
    double output = clamp(-0.9, 0.9, (center * Constants.kTurretP)) * Constants.kMaxVel;

    double[] t = deadeye.getTranslationFromCamCenter();

    double height = t[1];
    double depth = t[2];

    double servoOut = clamp(0.01, 0.99, Math.atan2(height, depth) / Math.PI + 5.0 / 180.0 + 0.6);

    turret.set(TalonSRXControlMode.Velocity, output);
    if (depth > 0) {
      pointer.set(servoOut);
    }
    if (deadeye.seesTarget()) {
      if (ledSubsystem.getState() != LedState.CANDY) ledSubsystem.setCandy();
    } else {
      if (ledSubsystem.getState() != LedState.FLAMING) ledSubsystem.setFlaming();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    System.out.println(wallEYE.updateNumber());
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
