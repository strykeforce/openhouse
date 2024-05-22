package frc.robot;

import WallEye.WallEyeCam;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class wallEYESubsystem extends MeasurableSubsystem {
  WallEyeCam cam;

  public wallEYESubsystem() {
    cam = new WallEyeCam("shooter", 0, -1);
  }

  @Override
  public void periodic() {
    cam.getResults();
  }

  public String numTargets() {
    return cam.getUDPnumTargets();
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return null;
  }
}
