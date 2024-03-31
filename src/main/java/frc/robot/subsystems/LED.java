// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class LED extends SubsystemBase {
  private static final String ROOT_TABLE = "LED";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.EnumValue<RobotState> log_robotState =
      logGroup.buildEnum("robotState");
  private static final LoggerEntry.EnumValue<BaseRobotState> log_baseRobotState =
      logGroup.buildEnum("baseRobotState");
  private static final LoggerEntry.Bool log_noteInRobot = logGroup.buildBoolean("noteInRobot");
  private static final LoggerEntry.Bool log_isTeleop = logGroup.buildBoolean("isTeleop");

  /** Creates a new LED. */
  private AddressableLED led = new AddressableLED(Constants.LEDConstants.PWM_PORT);

  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(13);
  private AddressableLEDBuffer previousBuffer = new AddressableLEDBuffer(13);

  private int snakeShade = 0;
  private int rainbowFirstPixelHue = 0;
  private int levelMeterCount = 0;
  private RobotState robotState = RobotState.BASE;
  private BaseRobotState baseRobotState = BaseRobotState.NOTE_STATUS;
  private boolean noteInRobot;
  private Color squirrelOrange = new Color(1, 0.1, 0);
  private TunableNumberGroup group = new TunableNumberGroup("LED");
  private LoggedTunableNumber useTunableLEDs = group.build("useTunableLEDs", 0);
  private LoggedTunableNumber tunableR = group.build("tunableColor/r", 0);
  private LoggedTunableNumber tunableG = group.build("tunableColor/g", 0);
  private LoggedTunableNumber tunableB = group.build("tunableColor/b", 0);
  private final DoubleSupplier elevatorHeight;

  public LED(DoubleSupplier elevatorHeight) {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
    this.elevatorHeight = elevatorHeight;
  }

  @Override
  public void periodic() {

    if (useTunableLEDs.get() == 0) {
      // This method will be called once per scheduler run
      switch (robotState) {
        case BASE:
          switch (baseRobotState) {
            case NOTE_STATUS:
              if (DriverStation.isDisabled() && elevatorHeight.getAsDouble() < 1) {
                setSnake2(Color.kWhite, Color.kMagenta);
              } else {
                if (noteInRobot) {
                  setSolidColor(squirrelOrange);

                } else {

                  if (DriverStation.isTeleop() && DriverStation.isEnabled()) {
                    setSolidColor(Color.kBlack);
                  } else if (DriverStation.isAutonomous()) {
                    setAudioLevelMeter(100);
                  } else {
                    setSnake2(squirrelOrange, new Color(1, 0.3, 0));
                  }
                }
                break;
              }
            case AUTO_NOTE_PICKUP:
              setSolidColor(Color.kMagenta);
              break;

            case AUTO_DRIVE_TO_POSE:
              setSolidColor(Color.kYellow);

            case AMP_LINE_UP:
              setBlinking(Color.kWhite, Color.kGreen);
              break;

            case CLIMB_LINE_UP:
              setBlinking(Color.kWhite, Color.kBlue);
              break;

            case SHOOTING_PREP:
              setSolidColor(Color.kYellow);
              break;
            case SHOOTER_SUCCESS:
              setSolidColor(Color.kGreen);
              break;

            default:
              setAudioLevelMeter(100);
              break;
          }
          break;

        case AMP_READY_TO_SCORE:
          setSolidColor(Color.kGreen);
          break;
        case TWENTY_SECOND_WARNING:
          setBlinking(Color.kMagenta, Color.kBlack);
          break;
        case HOME_SUBSYSTEMS:
          setBlinking(Color.kGreen, Color.kBlack);
          break;
        case BREAK_MODE_ON:
          setBlinking(Color.kRed, Color.kBlack);
          break;
        case BREAK_MODE_OFF:
          setBlinking(Color.kBlue, Color.kBlack, 0.3);
          break;
        case TEST:
          setSolidColor(Color.kCyan);
          break;
        case INTAKE_SUCCESS:
          setBlinking(Color.kGreen, Color.kBlack);
          break;
      }
    } else {
      setSnake2(new Color(tunableR.get(), tunableG.get(), tunableB.get()), Color.kRed);
    }

    log_robotState.info(robotState);
    log_baseRobotState.info(baseRobotState);
    log_noteInRobot.info(noteInRobot);
    log_isTeleop.info(DriverStation.isTeleop());

    if (!sameAsPrevBuffer()) led.setData(ledBuffer);

    ledBuffer.forEach(
        (i, r, g, b) -> {
          previousBuffer.setRGB(i, r, g, b);
        });
  }

  private void setSolidColor(Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
  }

  private void setBlinking(Color color1, Color color2) {
    setBlinking(color1, color2, 0.1);
  }

  private void setBlinking(Color color1, Color color2, double period) {

    if (Math.sin(Timer.getFPGATimestamp() * Math.PI / period) >= 0) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, color1);
      }
    } else {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, color2);
      }
    }
  }

  private void setSnake(Color color1, Color color2) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final var shade = (snakeShade + (i * 255 / ledBuffer.getLength())) % 255;
      ledBuffer.setRGB(
          i,
          (int) (shade * color1.red + color2.red * (255 - shade)),
          (int) (shade * color1.green + color2.green * (255 - shade)),
          (int) (shade * color1.blue + color2.blue * (255 - shade)));

      snakeShade += 0.5;
      snakeShade %= 255;
    }
  }

  private void setSnake2(Color color1, Color color2) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final var shade = Math.sin(Timer.getFPGATimestamp() * 4 - i * 0.32);
      ledBuffer.setRGB(
          i,
          (int) (shade * color1.red * 255 + color2.red * (1 - shade) * 255),
          (int) (shade * color1.green * 255 + color2.green * (1 - shade) * 255),
          (int) (shade * color1.blue * 255 + color2.blue * (1 - shade)) * 255);
    }
  }

  private void setRainbow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  public void setRobotState(RobotState robotState) {
    this.robotState = robotState;
  }

  /**
   * setAudioLevelMeter() - looks like an audio level meter
   *
   * @param bpm beats per minute
   */
  private void setAudioLevelMeter(int bpm) {

    int max = ledBuffer.getLength();
    double theta = levelMeterCount * 0.02 * Math.PI * bpm / 60.0;
    int volume =
        (int)
            Math.round(
                1
                    + Math.abs(11.0 * Math.sin(theta))
                    + (3.0 * Math.sin(theta * 7.0))
                    + (1.0 * Math.sin(theta * 17.0)));

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i <= volume) {
        if (i <= (0.6 * max)) {
          ledBuffer.setLED(i, Color.kGreen);
        } else if (i <= 0.8 * max) {
          ledBuffer.setLED(i, Color.kYellow);
        } else {
          ledBuffer.setLED(i, Color.kRed);
        }
      } else {
        ledBuffer.setLED(i, Color.kBlack);
      }
    }
    levelMeterCount += 1;

    // System.out.println("LED: count=" + levelMeterCount + " vol=" + volume + " theta=" + theta);
  }

  private void setNothing() {
    setSolidColor(Color.kBlack);
  }

  public RobotState getCurrentState() {
    return robotState;
  }

  public void setBaseRobotState(BaseRobotState baseRobotState) {
    this.baseRobotState = baseRobotState;
  }

  public BaseRobotState getCurrentBaseState() {
    return baseRobotState;
  }

  public void setNoteStatus(boolean noteInRobot) {
    this.noteInRobot = noteInRobot;
  }

  public boolean getNoteStatus() {
    return noteInRobot;
  }

  public enum RobotState {
    TWENTY_SECOND_WARNING,
    AMP_READY_TO_SCORE,
    TEST,
    HOME_SUBSYSTEMS,
    BREAK_MODE_OFF,
    BREAK_MODE_ON,
    BASE,
    INTAKE_SUCCESS
  }

  public enum BaseRobotState {
    NOTE_STATUS,
    AUTO_NOTE_PICKUP,
    AUTO_DRIVE_TO_POSE,
    AMP_LINE_UP,
    CLIMB_LINE_UP,
    SHOOTING_PREP,
    SHOOTER_SUCCESS
  }

  public boolean sameAsPrevBuffer() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (!ledBuffer.getLED(i).equals(previousBuffer.getLED(i))) return false;
    }

    return true;
  }
}
