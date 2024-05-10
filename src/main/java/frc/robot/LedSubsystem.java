package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LedSubsystem extends MeasurableSubsystem {

  private LedState currState = LedState.OFF;
  private int flameCounter = 0;
  private int blinkCounter = 0; // 0 -> kBlinkOffCount -> kBlinkOnCount -> 0
  private boolean blinking = false;

  private Color currColor = new Color();

  private AddressableLED ledR = new AddressableLED(Constants.kLedPort);
  private AddressableLEDBuffer ledBufferR = new AddressableLEDBuffer(Constants.kLedLen);
  private int loopCounter = 0;
  private int cycleCenter = 0;

  public LedSubsystem() {
    ledR.setLength(ledBufferR.getLength());
    ledR.setData(ledBufferR);
    ledR.start();
  }

  public LedState getState() {
    return currState;
  }

  public void test() {
    for (int i = 0; i < Constants.kLedLen; i++) {
        System.out.println(new Color(
              ((double) i) / Constants.kLedLen,
              ((double) i) / Constants.kLedLen,
              ((double) i) / Constants.kLedLen));
      setLED(
          i,
          new Color(
              ((double) i) / Constants.kLedLen,
              ((double) i) / Constants.kLedLen,
              ((double) i) / Constants.kLedLen));
    }
    ledR.setData(ledBufferR);
  }

  private void setState(LedState state) {
    currState = state;
  }

  public void setBlinking(boolean blinking) {
    this.blinking = blinking;
  }

  // this has no data setting! only sets the buffer!
  private void setLED(int i, int r, int g, int b) {
    ledBufferR.setRGB(i, g, r, b);
  }

  // this has no data setting! only sets the buffer!
  private void setLED(int i, Color color) {
    ledBufferR.setRGB(
        i, (int) (color.green * 255.0), (int) (color.red * 255.0), (int) (color.blue * 255.0));
  }

  public void setColor(int r, int g, int b) {
    setState(LedState.SOLID);
    for (var i = 0; i < ledBufferR.getLength(); i++) {
      setLED(i, r, g, b);
    }
    ledR.setData(ledBufferR);
  }

  public void setColor(Color color) {
    currColor = color;
    setState(LedState.SOLID);
    for (var i = 0; i < ledBufferR.getLength(); i++) {
      setLED(i, color);
    }
    ledR.setData(ledBufferR);
  }

  public void blinkOff() {
    for (var i = 0; i < ledBufferR.getLength(); i++) {
      setLED(i, 0, 0, 0);
    }
    ledR.setData(ledBufferR);
  }

  public void setOff() {
    setColor(new Color());
    currState = LedState.OFF;
  }

  public double cycleBrightness(int led, int center) {
    return 2.0 / (1 + Math.exp((led - center) * (led - center) / 6.0));
  }

  @Override
  public void periodic() {
    switch (currState) {
      case CYCLE:
        for (int i = 0; i < Constants.kLedLen; i++) {
          double factor = cycleBrightness(i, cycleCenter);
          setLED(
              i,
              (int) (currColor.red * 255 * factor),
              (int) (currColor.blue * 255 * factor),
              (int) (currColor.green * 255 * factor));
        }

        break;

      case SOLID:
        break;
      default:
        break;
    }
    loopCounter++;
    if (loopCounter > 20) {
        loopCounter = 0;
        cycleCenter++;
        cycleCenter %= Constants.kLedLen;
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("State", () -> getState().ordinal()),
        new Measure("is Blinking", () -> blinking ? 1.0 : 0.0),
        new Measure("Blink Count", () -> blinkCounter));
  }

  public enum LedState {
    OFF,
    SOLID,
    CYCLE
  }
}
