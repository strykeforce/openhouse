package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intake;
  private boolean isIntaking = false;

  public IntakeSubsystem() {
    intake = new TalonFX(Constants.kIntakeId);
    intake.getConfigurator().apply(Constants.getIntakeConfig());
  }

  public void toggleIntaking() {
    if (isIntaking) stop();
    else runIntake();
  }

  public void stop() {
    isIntaking = false;
    intake.set(0);
  }

  public void runIntake() {
    isIntaking = true;
    intake.set(Constants.kIntakeSpeed);
  }
}
