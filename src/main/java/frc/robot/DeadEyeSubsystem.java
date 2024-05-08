package frc.robot;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Set;
import org.ejml.simple.SimpleMatrix;
import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.deadeye.Rect;
import org.strykeforce.deadeye.TargetListTargetData;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DeadEyeSubsystem extends MeasurableSubsystem {

  Deadeye<TargetListTargetData> cam;
  double centerX = 0.0;
  TargetListTargetData data = new TargetListTargetData();

  public DeadEyeSubsystem() {
    cam = new Deadeye<>("W0", TargetListTargetData.class);
    centerX = cam.getCapture().width / 2.0;
  }

  public boolean isCamEnabled() {
    return cam.getEnabled();
  }

  public void setCamEnabled(boolean val) {
    cam.setEnabled(val);
  }

  public TargetListTargetData getTargetListData() {
    return data;
  }

  public double getDistanceToCamCenter() {
    if (data.targets.size() == 0) {
      return 0.0;
    }

    ArrayList<Rect> validTargs = new ArrayList<Rect>();

    for (Rect targ : data.targets) {
      // data.targetsOrderedByBottomRightX()
      validTargs.add(targ);
    }
    validTargs.sort(
        Comparator.comparingInt(r -> -((Rect) r).center().y)
            .thenComparingInt(r -> Math.abs(((int) centerX - ((Rect) r).center().x))));

    if (validTargs.size() > 0) return validTargs.get(0).center().x - centerX;
    return 0.0;
  }

  public double[] getTranslationFromCamCenter() {
    double[] ret = {0, 0, 0};
    if (data.targets.size() == 0) {
      return ret;
    }

    ArrayList<Rect> validTargs = new ArrayList<Rect>();

    for (Rect targ : data.targets) {
      // data.targetsOrderedByBottomRightX()
      validTargs.add(targ);
    }

    validTargs.sort(
        Comparator.comparingInt(r -> ((Rect) r).size()).thenComparingInt(r -> ((Rect) r).size()));

    if (validTargs.size() > 0) {
      double[] a = {validTargs.get(0).topLeft.x, validTargs.get(0).topLeft.y};
      double[] b = {validTargs.get(0).bottomRight.x, validTargs.get(0).bottomRight.y};

      double[] t = get3D(a, b);
      return t;
    }
    return ret;
  }

  private double getTargetY() {
    double lastValidTargY = 2767.0;
    for (Rect targ : data.targets) {
      lastValidTargY = targ.center().y;
    }
    return lastValidTargY;
  }

  public int getNumTargets() {
    return data.targets.size();
  }

  // public boolean isNoteRight() {
  // List<Rect> list = data.targets;

  // for (Rect rect : list) {
  // if (rect.topLeft.x >= border2) return true;
  // }
  // return false;
  // }

  // public boolean isNoteMid() {
  // List<Rect> list = data.targets;

  // for (Rect rect : list) {
  // if (rect.topLeft.x >= border1 && rect.bottomRight.x <= border2) return true;
  // }
  // return false;
  // }

  // public boolean isNoteLeft() {
  // List<Rect> list = data.targets;

  // for (Rect rect : list) {
  // if (rect.bottomRight.x <= border1) return true;
  // }
  // return false;
  // }

  // NOT PART OF DEADEYE - OPEN HOUSE DEMO ONLY
  public double[] get3D(double[] apogee1, double[] apogee2) {

    double[][] eye = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    double[][] K = {
      {1112.87614936, 0.0, 631.27084086}, {0.0, 1110.504635, 404.26599437}, {0.0, 0.0, 1.0}
    };

    SimpleMatrix r = new SimpleMatrix(eye);
    SimpleMatrix k = new SimpleMatrix(K);

    SimpleMatrix point2DHomogeneous = new SimpleMatrix(3, 2);
    point2DHomogeneous.setColumn(0, 0, apogee1[0], apogee1[1], 1.0);
    point2DHomogeneous.setColumn(1, 0, apogee2[0], apogee2[1], 1.0);

    SimpleMatrix rotMatrixInv = r.invert();
    SimpleMatrix kInv = k.invert();
    SimpleMatrix inverseRotAndK = rotMatrixInv.mult(kInv);

    SimpleMatrix point3dDirection1 =
        inverseRotAndK.mult(point2DHomogeneous.extractVector(false, 0));
    SimpleMatrix point3dDirection2 =
        inverseRotAndK.mult(point2DHomogeneous.extractVector(false, 1));

    double scaleFactor = 35.56 / point3dDirection1.minus(point3dDirection2).normF();
    SimpleMatrix endpoints = point3dDirection1.combine(0, 1, point3dDirection2).scale(scaleFactor);

    double[] result = new double[3];
    result[0] = (endpoints.get(0) + endpoints.get(3)) / 2;
    result[1] = (endpoints.get(1) + endpoints.get(4)) / 2;
    result[2] = (endpoints.get(2) + endpoints.get(5)) / 2;

    return result;
  }

  @Override
  public void periodic() {
    TargetListTargetData newData = cam.getTargetData();
    // System.out.println(newData == null ? "NULL" : newData);
    if (newData != null) {
      data = newData;
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Is deadeye enabled", () -> cam.getEnabled() ? 1 : 0),
        new Measure("Y Pixel", () -> getTargetY()));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }
}
