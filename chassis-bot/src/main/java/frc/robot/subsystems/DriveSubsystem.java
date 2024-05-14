package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.strykeforce.gyro.SF_AHRS;
import org.strykeforce.swerve.PoseEstimatorOdometryStrategy;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.V6TalonSwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule.ClosedLoopUnits;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  private PoseEstimatorOdometryStrategy odometryStrategy;
  private V6TalonSwerveModule[] swerveModules;
  private SwerveDriveKinematics kinematics;
  private TalonFXConfigurator configurator;

  private double fwdStrScale = Constants.kFwdStrScale;
  private double yawScale = Constants.kYawScale;
  private boolean isSlow = true;

  private SF_AHRS ahrs;

  public DriveSubsystem() {
    var moduleBuilder =
        new V6TalonSwerveModule.V6Builder()
            .driveGearRatio(Constants.kDriveGearRatio)
            .wheelDiameterInches(Constants.kWheelDiameterInches)
            .driveMaximumMetersPerSecond(Constants.kMaxSpeedMetersPerSecond)
            .latencyCompensation(true);

    swerveModules = new V6TalonSwerveModule[4];
    Translation2d[] wheelLocations = Constants.getWheelLocationMeters();

    for (int i = 0; i < 4; i++) {
      var azimuthTalon = new TalonSRX(i);
      azimuthTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
      azimuthTalon.configAllSettings(
          Constants.getAzimuthTalonConfig(), Constants.kTalonConfigTimeout);
      azimuthTalon.enableCurrentLimit(true);
      azimuthTalon.enableVoltageCompensation(true);
      azimuthTalon.setNeutralMode(NeutralMode.Coast);

      var driveTalon = new TalonFX(i + 10);
      configurator = driveTalon.getConfigurator();
      configurator.apply(new TalonFXConfiguration());
      configurator.apply(Constants.getDriveTalonConfig());

      swerveModules[i] =
          moduleBuilder
              .azimuthTalon(azimuthTalon)
              .driveTalon(driveTalon)
              .wheelLocationMeters(wheelLocations[i])
              .closedLoopUnits(ClosedLoopUnits.VOLTAGE)
              .build();
      swerveModules[i].loadAndSetAzimuthZeroReference();
    }

    ahrs = new SF_AHRS(SPI.Port.kMXP, 2_000_000, (byte) 200);
    swerveDrive = new SwerveDrive(false, 0.02, ahrs, swerveModules);
    swerveDrive.resetGyro();
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));
  }

  public void drive(double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {
    swerveDrive.drive(vXmps, vYmps, vOmegaRadps, isFieldOriented);
  }

  public void resetGyro() {
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));
    swerveDrive.resetGyro();
  }

  public void toggleSlow() {
    if (isSlow) setSlow(false);
    else setSlow(true);
  }

  public void setSlow(boolean slow) {
    isSlow = slow;
    if (slow) {
      fwdStrScale = Constants.kFwdStrScale;
      yawScale = Constants.kYawScale;
    } else {
      fwdStrScale = 1.0;
      yawScale = 1.0;
    }
  }

  public boolean isSlow() {
    return isSlow;
  }

  public double getFwdStrScale() {
    return fwdStrScale;
  }

  public double getYawScale() {
    return yawScale;
  }
}
