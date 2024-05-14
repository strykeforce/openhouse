package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
  public static final int kTalonConfigTimeout = 10;
  public static final double kDeadbandAllStick = 0.075;
  public static final double kRateLimitFwdStr = 3.5;
  public static final double kRateLimitYaw = 8.0;
  public static final double kFwdStrScale = 1.0;
  public static final double kYawScale = 1.0;

  public static final double kRobotLength = 0.5461;
  public static final double kRobotWidth = 0.6922;
  public static final double kWheelDiameterInches = 3.0 * 502.0 / 500.0;

  public static final double kDriveMotorOutputGear = 34; // 30
  public static final double kDriveInputGear = 42;
  public static final double kBevelInputGear = 15;
  public static final double kBevelOutputGear = 45;

  public static final double kDriveGearRatio =
      (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);
  //   public static final double kWheelDiameterInches = 3.0 * 506.0 / 500.0;
  public static final double kMaxSpeedMetersPerSecond = 6.495;

  public static final double kMaxOmega =
      (kMaxSpeedMetersPerSecond / Math.hypot(kRobotWidth / 2.0, kRobotLength / 2.0))
          / 2.0; // wheel locations below

  public static Translation2d[] getWheelLocationMeters() {
    final double x = kRobotLength / 2.0; // front-back, was ROBOT_LENGTH
    final double y = kRobotWidth / 2.0; // left-right, was ROBOT_WIDTH
    Translation2d[] locs = new Translation2d[4];
    locs[0] = new Translation2d(x, y); // left front
    locs[1] = new Translation2d(x, -y); // right front
    locs[2] = new Translation2d(-x, y); // left rear
    locs[3] = new Translation2d(-x, -y); // right rear
    return locs;
  }

  public static TalonSRXConfiguration getAzimuthTalonConfig() {
    // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
    TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();

    azimuthConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    azimuthConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

    azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
    azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

    azimuthConfig.continuousCurrentLimit = 10;
    azimuthConfig.peakCurrentDuration = 0;
    azimuthConfig.peakCurrentLimit = 0;

    azimuthConfig.slot0.kP = 15.0;
    azimuthConfig.slot0.kI = 0.0;
    azimuthConfig.slot0.kD = 150.0;
    azimuthConfig.slot0.kF = 1.0;
    azimuthConfig.slot0.integralZone = 0;
    azimuthConfig.slot0.allowableClosedloopError = 0;
    azimuthConfig.slot0.maxIntegralAccumulator = 0;

    azimuthConfig.motionCruiseVelocity = 800;
    azimuthConfig.motionAcceleration = 10_000;
    azimuthConfig.velocityMeasurementWindow = 64;
    azimuthConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    azimuthConfig.voltageCompSaturation = 12;
    azimuthConfig.voltageMeasurementFilter = 32;
    azimuthConfig.neutralDeadband = 0.04;
    return azimuthConfig;
  }

  // Drive Falcon Config
  public static TalonFXConfiguration getDriveTalonConfig() {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 45; // 40
    currentConfig.SupplyCurrentThreshold = 50; // 45
    currentConfig.SupplyTimeThreshold = 0.0;
    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = false;
    driveConfig.CurrentLimits = currentConfig;

    Slot0Configs slot0Config = new Slot0Configs();
    slot0Config.kP = 0.5; // 0.16 using phoenix 6 migrate
    slot0Config.kI = 0.5; // 0.0002 using phoenix 6 migrate
    slot0Config.kD = 0.0;
    slot0Config.kV = 0.12; // 0.047 using phoenix 6 migrate
    driveConfig.Slot0 = slot0Config;

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.DutyCycleNeutralDeadband = 0.01;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput = motorConfigs;

    return driveConfig;
  }
}
