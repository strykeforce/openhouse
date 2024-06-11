package frc.robot;

import WallEye.WallEyeCam;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class wallEYESubsystem extends MeasurableSubsystem {
  WallEyeCam cam;

  public wallEYESubsystem() {
    cam = new WallEyeCam("Walleye", 0, 5802, -1);
  }

  @Override
  public void periodic() {
    cam.getResults();
  }

  public int updateNumber() {
    return cam.getResults().getUpdateNum();
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return null;
  }
}
