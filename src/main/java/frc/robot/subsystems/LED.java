// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AddressableLED led = new AddressableLED(Constants.LEDConstants.PWM_PORT);

  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(13);
  AddressableLEDBuffer previousBuffer = new AddressableLEDBuffer(13);

  int snakeShade = 0;
  int levelMeterCount = 0;
  int rainbowFirstPixelHue = 0;

  private RobotState robotState = RobotState.BASE;
  private BaseRobotState baseRobotState = BaseRobotState.NOTE_STATUS;
  private boolean noteInRobot;

  public LED() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (robotState) {
      case BASE:
        switch (baseRobotState) {
          case NOTE_STATUS:
            if (noteInRobot) {
              setSolidColor(new Color(255, 45, 0));

            } else {
              setSolidColor(Color.kBlack);
            }
            break;

          case AUTO_NOTE_PICKUP:
            setBlinking(Color.kWhite, Color.kBlack);
            break;

          case AMP_LINE_UP:
            setBlinking(Color.kWhite, Color.kGreen);
            break;

          case CLIMB_LINE_UP:
            setBlinking(Color.kWhite, Color.kBlue);
            break;

          case SHOOTING_PREP:
            setBlinking(Color.kWhite, Color.kRED);
            break;
          case SHOOTER_SUCCESS:
            setSolidColor(Color.kGreen);
            break;

          default:
            break;
        }
        break;

      case TEST:
      case SHOOTER_LINING_UP:
      case DEFAULT:
        setAudioLevelMeter(100);
        // setSolidColor(new Color(255, 45, 0)); // set solid orange
        break;
      case SHOOTER_SUCCESS:
        // test writing solid color
        // FIXME: if wanted, inside of setSolidColor could remove parameters once we have certain
        // values we want to use
        setSolidColor(Color.kGreen);
        break;
      case SHOOTER_LINED_UP:
        // test of writing blinking
        // FIXME: if wanted, inside of setBlinking could remove paramters once we have certain
        // values we want to use
        setBlinking(Color.kBlack, Color.kWhite);
        break;
      case AMP_READY_TO_SCORE:
        setSolidColor(Color.kGreen);
        break;
      case DRIVING_TO_GAMEPIECE:
        setAllRainbow();
        break;
      case AUTO_MODE:
        setSolidColor(Color.kWhite);
        break;
      case NOTHING:
        setSolidColor(Color.kBlack);
        break;
      case TWENTY_SECOND_WARNING:
        // when the match has 20 seconds left this code will change the color to magenta
        setBlinking(Color.kMagenta, Color.kBlack);
        break;
      case AMP_LINING_UP:
        setBlinking(Color.kYellow, Color.kBlack);
      case GAMEPIECE_IN_ROBOT:
        setSolidColor(Color.kOrange);
        break;
      case CLIMB_MODE:
        setSolidColor(Color.kGreen);
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
    }

    Logger.recordOutput("LED/robotState", robotState);
    Logger.recordOutput("LED/baseRobotState", baseRobotState);
    Logger.recordOutput("LED/noteInRobot", noteInRobot);

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

  private void setAllSnake(Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final var shade = (snakeShade + (i * 255 / ledBuffer.getLength())) % 255;
      ledBuffer.setRGB(
          i, (int) (shade * color.red), (int) (shade * color.green), (int) (shade * color.blue));

      snakeShade += 3;
      snakeShade %= 255;
    }
  }

  private void setAllRainbow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
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

  private class individualLED {
    int start;
    int end;

    public individualLED(int start, int end) {
      this.start = start;
      this.end = end;
    }

    public int getLength() {
      if (start == 0) {
        return Math.abs(end - start);
      } else {
        return Math.abs(end - start) + 1;
      }
    }
  }

  public boolean sameAsPrevBuffer() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (!ledBuffer.getLED(i).equals(previousBuffer.getLED(i))) return false;
    }

    return true;
  }

  public robotStates getCurrentState() {
    return robotState;
  }

  public enum robotStates {
    SHOOTER_SUCCESS,
    SHOOTER_LINED_UP,
    DRIVING_TO_GAMEPIECE,
    AUTO_MODE,
    NOTHING,
    TWENTY_SECOND_WARNING,
    AMP_LINING_UP,
    SHOOTER_LINING_UP,
    AMP_READY_TO_SCORE,
    GAMEPIECE_IN_ROBOT,
    CLIMB_MODE,
    TEST,
    HOME_SUBSYSTEMS,
    BREAK_MODE_OFF,
    BREAK_MODE_ON,
    DEFAULT
  }

  public void setRobotState(robotStates robotState) {
    this.robotState = robotState;
  }
}
