package frc.robot;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class Constants {
  public static final int kServoID = 0;
  public static final int kFalconSRXID = 6;
  public static final int kLedPort = 1;
  public static final int kLedLen = 41;

  public static final double kMaxVel = 290.0;

  public static final double kTurretP = 0.001;
  public static final double kTurretI = 0;
  public static final double kTurretD = 0;

  public static TalonSRXConfiguration getSrxConfiguration() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.forwardSoftLimitThreshold = 1515;
    config.forwardSoftLimitEnable = true; // fixme

    config.reverseSoftLimitThreshold = -2200.0;
    config.reverseSoftLimitEnable = true; // fixme

    config.continuousCurrentLimit = 10;
    config.peakCurrentLimit = 15;
    config.peakCurrentDuration = 40;

    config.slot0.kP = 0.5;
    config.slot0.kI = 0.01;
    config.slot0.kD = 0.0;
    config.slot0.kF = 3.5;
    config.slot0.integralZone = 10;
    config.slot0.maxIntegralAccumulator = 1000;
    config.slot0.allowableClosedloopError = 0;
    config.motionCruiseVelocity = 600;
    config.motionAcceleration = 2000;
    config.neutralDeadband = 0.01;
    config.velocityMeasurementWindow = 64;
    config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;

    config.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
    config.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

    return config;
  }
}
