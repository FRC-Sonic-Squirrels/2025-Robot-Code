// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.function.Supplier;

public class LED extends SubsystemBase {
  private static final String ROOT_TABLE = "LED";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.EnumValue<RobotState> log_robotState =
      logGroup.buildEnum("robotState");
  private static final LoggerEntry.EnumValue<BaseRobotState> log_baseRobotState =
      logGroup.buildEnum("baseRobotState");
  private static final LoggerEntry.Bool log_gamepieceInRobot =
      logGroup.buildBoolean("gamepieceInRobot");
  private static final LoggerEntry.Bool log_isTeleop = logGroup.buildBoolean("isTeleop");

  /** Creates a new LED. */
  private AddressableLED led = new AddressableLED(Constants.LEDConstants.PWM_PORT);

  private AddressableLEDBuffer ledBuffer =
      new AddressableLEDBuffer(13); // TODO: change length of buffers to new
  // robot's led size
  private AddressableLEDBuffer previousBuffer = new AddressableLEDBuffer(13);

  private final Distance ledSpacing = Meters.of(1 / 120.0);

  private int levelMeterCount = 0;
  private RobotState robotState = RobotState.BASE;
  private BaseRobotState baseRobotState = BaseRobotState.GAMEPIECE_STATUS;
  private boolean gamepieceInRobot;
  private Color squirrelOrange = new Color(1, 0.1, 0);
  private TunableNumberGroup group = new TunableNumberGroup("LED");
  private LoggedTunableNumber useTunableLEDs = group.build("useTunableLEDs", 0);
  private LoggedTunableNumber tunableR = group.build("tunableColor/r", 0);
  private LoggedTunableNumber tunableG = group.build("tunableColor/g", 0);
  private LoggedTunableNumber tunableB = group.build("tunableColor/b", 0);
  private int robotLoops = 0;
  private final int robotLoopsTillReady = 20;
  private final Supplier<Boolean> brakeMode;
  private final Supplier<Boolean> gyroConnected;

  public LED(Supplier<Boolean> brakeMode, Supplier<Boolean> gyroConnected) {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
    this.brakeMode = brakeMode;
    this.gyroConnected = gyroConnected;
  }

  @Override
  public void periodic() {
    robotLoops++;

    if (useTunableLEDs.get() == 0) {
      // This method will be called once per scheduler run
      // TODO: add condition for if elevator is not zeroed.

      switch (robotState) {
        case BASE:
          switch (baseRobotState) {
            case GAMEPIECE_STATUS:
              if (robotLoops < robotLoopsTillReady) {
                setProgressBar(Color.kGreen, (double) robotLoops / (double) robotLoopsTillReady);
              } else if (!brakeMode.get()) {
                setSnake(Color.kGreen, Color.kCrimson);
              } else if (!gyroConnected.get() && !Constants.RobotMode.isSimBot()) {
                setBlinking(Color.kAquamarine);
              } else {
                if (gamepieceInRobot) {
                  setSolidColor(squirrelOrange);
                } else {
                  if (DriverStation.isTeleop() && DriverStation.isEnabled()) {
                    setSolidColor(Color.kBlack);
                  } else if (DriverStation.isAutonomous()) {
                    setSeaLevelGraphic();
                  } else {
                    setSnake(squirrelOrange, new Color(1, 0.3, 0));
                  }
                }
              }
              break;
            case AUTO_GAMEPIECE_PICKUP:
              setSolidColor(Color.kMagenta);
              break;

            case AUTO_DRIVE_TO_POSE:
              setSolidColor(Color.kYellow);

            case GOAL_LINE_UP:
              setBlinking(Color.kWhite);
              break;

            case SHOOTING_PREP:
              setSolidColor(Color.kYellow);
              break;
            case SHOOTER_SUCCESS:
              setSolidColor(Color.kBlueViolet);
              break;
            default:
              setSeaLevelGraphic();
              break;
          }
          break;

        case READY_TO_SCORE:
          setSolidColor(Color.kGreen);
          break;
        case TWENTY_SECOND_WARNING:
          setBlinking(Color.kMagenta);
          break;
        case HOME_SUBSYSTEMS:
          setBlinking(Color.kGreen);
          break;
        case BREAK_MODE_ON:
          setBlinking(Color.kRed);
          break;
        case BREAK_MODE_OFF:
          setBlinking(Color.kBlue, 0.3);
          break;
        case TEST:
          setSolidColor(Color.kCyan);
          break;
        case INTAKE_SUCCESS:
          setBlinking(Color.kGreen);
          break;
        case BRAKE_MODE_FAILED:
          setSolidColor(Color.kPurple);
          break;
        case NOT_ZEROED:
          setRainbow();
          break;
      }
    } else {
      setSnake(new Color(tunableR.get(), tunableG.get(), tunableB.get()), Color.kRed);
    }

    log_robotState.info(robotState);
    log_baseRobotState.info(baseRobotState);
    log_gamepieceInRobot.info(gamepieceInRobot);
    log_isTeleop.info(DriverStation.isTeleop());

    if (!sameAsPrevBuffer()) led.setData(ledBuffer);

    ledBuffer.forEach(
        (i, r, g, b) -> {
          previousBuffer.setRGB(i, r, g, b);
        });
  }

  // Setters

  /** setSolidColor() - The LEDs will show a static color. */
  private void setSolidColor(Color color) {
    LEDPattern solid = LEDPattern.solid(color);
    solid.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  /** setProgressBar() - A bottom-up progress bar will fill the LEDs with pure white at %100. */
  private void setProgressBar(Color color, double percent) {
    LEDPattern progress = LEDPattern.progressMaskLayer(() -> (percent) / 100);
    progress.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  /** setBlinking() - The LEDs will blink a color at the predefined interval of 0.1s */
  private void setBlinking(Color color) {
    setBlinking(color, 0.1);
  }

  /** setBlinking() - The LEDs will blink a color at a specific interval */
  private void setBlinking(Color color, double seconds) {
    LEDPattern solid = LEDPattern.solid(color);
    LEDPattern blink = solid.blink(Seconds.of(seconds));
    blink.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  /** setSnake() - Gradient of two colors that will scroll across the LEDs continously */
  private void setSnake(Color color1, Color color2) {
    LEDPattern gradient =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2)
            .scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);
    gradient.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  /** setRainbow() - Rainbow that will scroll across the LEDs continously */
  private void setRainbow() {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);
    scrollingRainbow.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  /** setRobotState() - Set the LED robotState. */
  public void setRobotState(RobotState robotState) {
    this.robotState = robotState;
  }

  /**
   * setAudioLevelMeter() - looks like an audio level meter. 2024 themed.
   *
   * @param bpm beats per minute
   */
  @SuppressWarnings("unused")
  private void setAudioLevelMeter(int bpm) {
    double theta = levelMeterCount * 0.02 * Math.PI * bpm / 60.0;
    double volume =
        1
            + Math.abs(11.0 * Math.sin(theta))
            + (3.0 * Math.sin(theta * 7.0))
            + (1.0 * Math.sin(theta * 17.0));
    double mappedVolume = (volume / 15) * 100;
    LEDPattern volumeMask = LEDPattern.progressMaskLayer(() -> (mappedVolume) / 100);
    LEDPattern volumeGradient =
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kRed)
            .mask(volumeMask);
    volumeGradient.applyTo(ledBuffer);
    led.setData(ledBuffer);
    levelMeterCount += 1;
  }

  /** setSeaLevelGraphic() - Looks like the side view of a changing sea level */
  private void setSeaLevelGraphic() {
    double seaLevel = 2 + Math.sin(levelMeterCount * .01) + Math.sin(levelMeterCount * .02);
    double mappedSeaLevel = (seaLevel / 4) * 100;
    LEDPattern seaMask = LEDPattern.progressMaskLayer(() -> (mappedSeaLevel) / 100);
    Color deepSea = new Color(0.0, 0.01, 0.025);
    Color shallowSea = new Color(0.0, 0.3, 0.4);
    Color seaColor = Color.lerpRGB(shallowSea, deepSea, mappedSeaLevel / 100);
    LEDPattern seaGradient =
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, deepSea, seaColor)
            .mask(seaMask);
    seaGradient.applyTo(ledBuffer);
    led.setData(ledBuffer);
    levelMeterCount += 1;
  }

  /** setBaseRobotState() - Needs an educated description. */
  // FIXME: Needs an educated description.
  public void setBaseRobotState(BaseRobotState baseRobotState) {
    this.baseRobotState = baseRobotState;
  }

  /** setGamepieceStatus() - Needs an educated description. */
  // FIXME: Needs an educated description.
  public void setGamepieceStatus(boolean gamepieceInRobot) {
    this.gamepieceInRobot = gamepieceInRobot;
  }

  // Getters

  public RobotState getCurrentState() {
    return robotState;
  }

  public BaseRobotState getCurrentBaseState() {
    return baseRobotState;
  }

  public boolean getGamepieceStatus() {
    return gamepieceInRobot;
  }

  public boolean sameAsPrevBuffer() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (!ledBuffer.getLED(i).equals(previousBuffer.getLED(i))) return false;
    }

    return true;
  }

  public enum RobotState {
    TWENTY_SECOND_WARNING,
    READY_TO_SCORE,
    TEST,
    HOME_SUBSYSTEMS,
    BREAK_MODE_OFF,
    BREAK_MODE_ON,
    BASE,
    INTAKE_SUCCESS,
    BRAKE_MODE_FAILED,
    NOT_ZEROED
  }

  public enum BaseRobotState {
    GAMEPIECE_STATUS,
    AUTO_GAMEPIECE_PICKUP,
    AUTO_DRIVE_TO_POSE,
    GOAL_LINE_UP,
    SHOOTING_PREP,
    SHOOTER_SUCCESS
  }
}
