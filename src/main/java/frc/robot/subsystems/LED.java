// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.awt.Color;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AddressableLED led = new AddressableLED(Constants.LEDConstants.PWM_PORT);

  individualLED led1 = new individualLED(0, 25);
  individualLED led2 = new individualLED(26, 60);

  int startLength;
  int endLength;

  // FIXME: these are unused but they might come in handy later
  Color color1 = Color.RED; // (255, 0, 0)
  Color color2 = Color.GREEN; // (0, 255, 0)
  Color color3 = Color.BLUE; // (0, 0 255)

  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(led1.getLength() + led2.getLength());
  int rainbowFirstPixelHue = 0;
  robotStates robotState = robotStates.GAMEPIECE_IN_ROBOT;

  public LED() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (robotState) {
      case SHOOTER_LINED_UP:
        // test writing solid color
        // FIXME: if wanted, inside of setallsolidcolor could remove parameters once we have certain
        // values we want to use
        setAllSolidColor(Color.GREEN);
        break;
      case SHOOTER_LINING_UP:
        // test of writing blinking
        // FIXME: if wanted, inside of setallblinking could remove paramters once we have certain
        // values we want to use
        setAllBlinking(Color.BLACK, Color.WHITE, 2);
        break;
      case AMP_READY_TO_SCORE:
        setAllSolidColor(Color.GREEN);
        break;
      case DRIVING_TO_GAMEPIECE:
        setAllRainbow();
        break;
      case AUTO_MODE:
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          ledBuffer.setRGB(i, Color.white.getRed(), Color.white.getGreen(), Color.white.getBlue());
        }
        break;
      case NOTHING:
        // when the case of the robot is nothing it will be set to red
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          ledBuffer.setRGB(i, Color.red.getRed(), 0, 0);
        }
        break;
      case TWENTY_SECOND_WARTING:
        // when the match has 20 seconds left this code will change the color to magenta
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          setAllBlinking(
              Color.magenta.getRed(), 0, Color.magenta.getGreen(), 0, Color.magenta.getBlue(), 0);
        }
        break;
      case AMP_LINING_UP:
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          setAllBlinking(
              Color.yellow.getRed(), 0, Color.yellow.getGreen(), 0, Color.yellow.getBlue(), 0);
      case GAMEPIECE_IN_ROBOT:
        setAllSolidColor(Color.ORANGE);
        break;
      case CLIMB_MODE:
        setSingleStripSolidColor(Color.GREEN, 20, 40);
        break;
      default:
        setNothing();
    }

    led.setData(ledBuffer);
  }

  private void setSingleStripSolidColor(Color color, int startingLED, int endingLED) {
    for (int i = startingLED; i <= endingLED; i++) {
      ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
    }
  }

  private void setAllSolidColor(Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
    }
  }

  private void setSingleStripBlinking(
      Color color1, Color color2, double seconds, int startingLED, int endingLED) {

    double period = (2 * Math.PI) / seconds;
    if (Math.sin(period * (Timer.getFPGATimestamp())) >= 0) {
      for (int i = startingLED; i <= endingLED; i++) {
        ledBuffer.setRGB(i, color1.getRed(), color1.getGreen(), color1.getBlue());
      }
    } else if (Math.sin(period * (Timer.getFPGATimestamp())) < 0) {
      for (int i = startingLED; i <= endingLED; i++) {
        ledBuffer.setRGB(i, color2.getRed(), color2.getGreen(), color2.getBlue());
      }
    }
  }

  private void setAllBlinking(Color color1, Color color2, double seconds) {
    double period = (2 * Math.PI) / seconds;
    if (Math.sin(period * (Timer.getFPGATimestamp())) >= 0) {
      for (int i = 0; i <= ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, color1.getRed(), color1.getGreen(), color1.getBlue());
      }
    } else if (Math.sin(period * (Timer.getFPGATimestamp())) < 0) {
      for (int i = 0; i <= ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, color2.getRed(), color2.getGreen(), color2.getBlue());
      }
    }
  }

  private void setSingleStripRainbow(int startingLED, int endingLED) {
    for (var i = startingLED; i <= endingLED; i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  private void setAllRainbow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  private void setNothing() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, Color.BLACK.getRed(), Color.BLACK.getGreen(), Color.BLACK.getBlue());
    }
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

  public enum robotStates {
    SHOOTER_SUCCESS(), //
    SHOOTER_LINED_UP(), //
    DRIVING_TO_GAMEPIECE(), //
    AUTO_MODE(), //
    NOTHING(), //
    TWENTY_SECOND_WARTING(), //
    AMP_LINING_UP(),
    SHOOTER_LINING_UP(),
    AMP_READY_TO_SCORE(),
    GAMEPIECE_IN_ROBOT(),
    CLIMB_MODE()
  }

  public void setRobotState(robotStates robotState) {
    this.robotState = robotState;
  }
}
