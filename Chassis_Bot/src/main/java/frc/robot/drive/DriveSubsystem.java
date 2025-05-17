package frc.robot.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.DriveConstants;
import java.util.Set;
import java.util.function.BooleanSupplier;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;


public class DriveSubsystem extends MeasurableSubsystem {
  private final SwerveIO io;
  private SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  public DriveStates currDriveState = DriveStates.IDLE;
  private ChassisSpeeds holoContOutput = new ChassisSpeeds();
  private double trajectoryActive = 0.0;

  private double driveMultiplier = 0.2;

  private int gyroDifferentCount = 0;
  private int gyroCorrectionCount = 0;

  private boolean ignoreSticks = false;
  private boolean boringDriving = true;

  public DriveSubsystem(SwerveIO io) {
    this.io = io;

  }

  public boolean hasZeroed() {
    return inputs.didZero;
  }

  public void zeroModules() {
    io.zeroModules();
  }

  // Open-Loop Swerve Movements
  public void drive(double vXmps, double vYmps, double vOmegaRadps) {
    // if (!ignoreSticks) {
      io.drive(vXmps * driveMultiplier, vYmps * driveMultiplier, vOmegaRadps, true);
    // }
  }

  public void toggleSafeDriving(){
    if(boringDriving){
        driveMultiplier = 1.0;
        boringDriving = false;
    }else{
        driveMultiplier = 0.2;
        boringDriving = true;
    }
  }

  public void stopDriving() {
    this.move(0, 0, 0, false);
    io.drive(0, 0, 0, false);
  }

  public void setAzimuthVel(double vel) {
    io.setAzimuthVel(vel);
  }

  // Closed-Loop (Velocity Controlled) Swerve Movement
  public void move(double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/Move X", vXmps);
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/Move Y", vYmps);
    org.littletonrobotics.junction.Logger.recordOutput("Swerve/Move Omega", vOmegaRadps);

    io.move(vXmps, vYmps, vOmegaRadps, isFieldOriented);
  }

  public void resetOdometry(Pose2d pose) {
    io.resetOdometry(pose);
  }

  public void setIgnoreSticks(boolean ignore) {
    org.littletonrobotics.junction.Logger.recordOutput("DriveSubsystem/Ignoring Sticks", ignore);
    this.ignoreSticks = ignore;
  }

  // Getters/Setters
  public Pose2d getPoseMeters() {
    return inputs.poseMeters;
  }

  public Rotation2d getGyroRotation2d() {
    return inputs.gyroRotation2d;
  }

  public ChassisSpeeds getFieldRelSpeed() {
    return inputs.fieldRelSpeed;
  }

  public ChassisSpeeds getRobotRelSpeed() {
    return inputs.robotRelSpeed;
  }

  public void setDriveState(DriveStates state) {
    currDriveState = state;
  }

  public BooleanSupplier getAzimuth1FwdLimitSupplier() {
    return io.getAzimuth1FwdLimitSwitch();
  }

  public boolean isDriveStill() {
    ChassisSpeeds cs = inputs.fieldRelSpeed;
    double vX = cs.vxMetersPerSecond;
    double vY = cs.vyMetersPerSecond;

    // Take fieldRel Speed and get the magnitude of the vector
    double wheelSpeed = Math.sqrt(FastMath.pow2(vX) + FastMath.pow2(vY));

    boolean velStill = Math.abs(wheelSpeed) <= DriveConstants.kSpeedStillThreshold;
    boolean gyroStill = isGyroStill();

    return velStill && gyroStill;
  }

  public boolean isGyroStill() {
    return Math.abs(inputs.gyroRate) <= DriveConstants.kGyroRateStillThreshold;
  }

  public void prepClimb() {
    io.setDriveCoast(true);
    io.setSwerveModuleAngles(
        Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(90));
  }

  public void teleResetGyro() {
    io.setBothGyroOffset(Rotation2d.fromDegrees(0));
    Logger.recordOutput("DriveSubsystem/gyroOffset", Rotation2d.fromDegrees(0));
    io.resetGyro();
    io.resetOdometry(
        new Pose2d(inputs.poseMeters.getTranslation(), Rotation2d.fromDegrees(0)));
  }

  // Make whether a trajectory is currently active obvious on grapher
  public void grapherTrajectoryActive(boolean active) {
    if (active) trajectoryActive = 1.0;
    else trajectoryActive = 0.0;
  }

  // Control Methods
  public void lockZero() {
    Rotation2d rot = Rotation2d.fromDegrees(0.0);
    io.setSwerveModuleAngles(rot, rot, rot, rot);
  }

  public void toSafeHold() {
    setDriveState(DriveStates.SAFE_HOLD);
    io.configDriveCurrents(DriveConstants.getSafeDriveLimits());
  }

  public void toIdle() {
    setDriveState(DriveStates.IDLE);
    io.configDriveCurrents(DriveConstants.getNormDriveLimits());
  }

  public double getAvgDriveCurrent() {
    return FastMath.abs(inputs.avgDriveCurrent);
  }

  public double getAvgRearDriveVel() {
    return inputs.avgRearDriveVel;
  }

  public void setDriveMultiplier(double multiplier) {
    driveMultiplier = multiplier;
  }

  public double getDriveMultiplier() {
    return driveMultiplier;
  }

  public void removeDriveMultiplier() {
    driveMultiplier = 1.0;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/Swerve Pose", inputs.swervePose);

    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/Gyro Correction Count", gyroCorrectionCount);

    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/Gyro Disagreement",
        inputs.gyroRotation2d.minus(inputs.navxRotation2d).getDegrees());

        org.littletonrobotics.junction.Logger.recordOutput(
          "DriveSubsystem/Drive Multipler",
          driveMultiplier);
          org.littletonrobotics.junction.Logger.recordOutput(
          "DriveSubsystem/Boring Driving",
          boringDriving);
    /* 
    if (Math.abs(inputs.gyroRotation2d.minus(inputs.navxRotation2d).getDegrees())
        > DriveConstants.kGyroDifferentThreshold) {
      gyroDifferentCount++;
    } else {
      gyroDifferentCount = 0;
    }
    if (gyroDifferentCount > DriveConstants.kGyroDifferentCount && isDriveStill()) {
      io.setPigeonGyroOffset(
          inputs.navxRotation2d.minus(inputs.gyroRotation2d).plus(io.getPigeonGyroOffset()));
      gyroDifferentCount = 0;
      gyroCorrectionCount++;
    }
      */

    switch (currDriveState) {
      case IDLE -> {}
      case SAFE -> {}
      case SAFE_HOLD -> {}
      default -> {}
    }
  }

  public enum DriveStates {
    IDLE,
    SAFE,
    SAFE_HOLD
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    io.registerWith(telemetryService);
    super.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("State", () -> currDriveState.ordinal()),
        new Measure("Gyro roll", () -> inputs.gyroRoll),
        new Measure("Gyro pitch", () -> inputs.gyroPitch),
        new Measure("Gyro Rotation2D(deg)", () -> inputs.gyroRotation2d.getDegrees()),
        new Measure("Odometry X", () -> inputs.poseMeters.getX()),
        new Measure("Odometry Y", () -> inputs.poseMeters.getY()),
        new Measure("Odometry Rotation2D(deg)", () -> inputs.poseMeters.getRotation().getDegrees()));
  }
}
