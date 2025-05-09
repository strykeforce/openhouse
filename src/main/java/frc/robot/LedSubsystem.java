package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LedSubsystem extends MeasurableSubsystem {

  private LedState currState = LedState.OFF;
  private int flameCounter = 0;

  private AddressableLED ledR = new AddressableLED(Constants.kRightLedPort);
  private AddressableLEDBuffer ledBufferR = new AddressableLEDBuffer(Constants.kRightLedLength);
  private int candyIterator = 0;
  private int loopCounter = 0;
  private LEDPattern alexa =
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlue);
  private double alexaTicks = 0.0;
  private double depth = 0.0;
  private Color alexaColor = new Color();

  //   private AddressableLED ledL = new AddressableLED(Constants.kLeftLedPort);
  //   private AddressableLEDBuffer ledBufferL;

  public LedSubsystem() {
    ledR.setLength(ledBufferR.getLength());
    ledR.setData(ledBufferR);
    ledR.start();

    // ledBufferL = new AddressableLEDBuffer(Constants.kLeftLedLength);
    // ledL.setLength(ledBufferL.getLength());
    // ledL.setData(ledBufferL);
    // ledL.start();
  }

  public LedState getState() {
    return currState;
  }

  private void setState(LedState state) {
    currState = state;
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
    // for (var i = 0; i < ledBufferL.getLength(); i++) {
    //   ledBufferL.setRGB(i, r, g, b);
    // }
    ledR.setData(ledBufferR);
    // ledL.setData(ledBufferL);
  }

  public void setColor(Color color) {
    setState(LedState.SOLID);
    for (var i = 0; i < ledBufferR.getLength(); i++) {
      setLED(i, color);
    }
    // for (var i = 0; i < ledBufferL.getLength(); i++) {
    //   ledBufferL.setLED(i, color);
    // }
    ledR.setData(ledBufferR);
    // ledL.setData(ledBufferL);
  }

  public void setFlaming() {
    setState(LedState.FLAMING);
  }

  public void setCandy() {
    setState(LedState.CANDY);
    candyIterator = 0;
  }

  public void setAlexa() {
    setState(LedState.ALEXA);
  }

  public void setOff() {
    setColor(new Color());
    currState = LedState.OFF;
  }

  public void setAlexaStuff(double PosTicks, double depth) {
    alexaTicks = PosTicks;
    this.depth = depth;
  }

  @Override
  public void periodic() {
    switch (currState) {
      case FLAMING:
        flameCounter++;
        if (flameCounter >= 3) {
          for (var i = 0; i < ledBufferR.getLength(); i++) {
            setLED(i, 250, (int) (Math.random() * 185), 0);
          }
          // for (var i = 0; i < ledBufferL.getLength(); i++) {
          //   ledBufferL.setRGB(i, 250, (int) (Math.random() * 185), 0);
          // }
          ledR.setData(ledBufferR);
          // ledL.setData(ledBufferL);
          flameCounter = 0;
        }

        break;
      case SOLID:
        break;
      case CANDY:
        if (loopCounter >= Constants.kLoopCounterCandy) {
          if (candyIterator >= Constants.candy.length - 1) {
            candyIterator = 0;
          } else {
            candyIterator++;
          }
          loopCounter = 0;
          for (var i = 0; i < ledBufferR.getLength(); i++) {
            setLED(i, Constants.candy[(i + candyIterator) % Constants.candy.length]);
          }
          ledR.setData(ledBufferR);
        } else {
          loopCounter++;
        }
        break;
      case ALEXA:
        alexaColor =
            new Color(
                0,
                MathUtil.clamp((depth - 60) / 140, 0, 1) * -1 + 1,
                MathUtil.clamp((depth - 60) / 140, 0, 1));
        LEDPattern alexa =
            LEDPattern.gradient(
                LEDPattern.GradientType.kContinuous,
                Color.kDarkGray,
                Color.kDarkGray,
                alexaColor,
                alexaColor);
        alexa = alexa.offsetBy((int) (Math.floor(alexaTicks * Constants.kTickToLED)) - 5);
        alexa.applyTo(ledBufferR);
        ledR.setData(ledBufferR);
        break;
      case OFF:
        break;
      default:
        setOff();
        break;
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("depth", () -> depth), new Measure("alexaColor blue", () -> alexaColor.blue));
  }

  public enum LedState {
    OFF,
    SOLID,
    FLAMING,
    CANDY,
    ALEXA
  }
}
