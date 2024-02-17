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
  AddressableLED leftLed = new AddressableLED(Constants.LEDConstants.LEFT_PWM_PORT);

  AddressableLED rightLed = new AddressableLED(Constants.LEDConstants.RIGHT_PWM_PORT);

  individualLED leftLed1 = new individualLED(0, 25);
  individualLED leftLed2 = new individualLED(26, 60);

  individualLED rightLed1 = new individualLED(0, 25);
  individualLED rightLed2 = new individualLED(26, 60);

  AddressableLEDBuffer leftLedBuffer =
      new AddressableLEDBuffer(leftLed1.getLength() + leftLed2.getLength());
  AddressableLEDBuffer rightLedBuffer =
      new AddressableLEDBuffer(rightLed1.getLength() + rightLed2.getLength());

  int snakeShade = 0;
  int rainbowFirstPixelHue = 0;
  robotStates robotState = robotStates.NOTHING;

  public LED() {
    leftLed.setLength(leftLedBuffer.getLength());
    leftLed.setData(leftLedBuffer);
    leftLed.start();

    rightLed.setLength(rightLedBuffer.getLength());
    rightLed.setData(rightLedBuffer);
    rightLed.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (robotState) {
      case SHOOTER_SUCCESS:
        // test writing solid color
        // FIXME: if wanted, inside of setallsolidcolor could remove parameters once we have certain
        // values we want to use
        setAllSolidColor(Color.RED, leftLedBuffer);
        break;
      case SHOOTER_LINED_UP:
        // test of writing blinking
        // FIXME: if wanted, inside of setallblinking could remove paramters once we have certain
        // values we want to use
        setAllBlinking(Color.BLACK, Color.WHITE, 0.5, leftLedBuffer);
        break;
      case DRIVING_TO_GAMEPIECE:
        setAllRainbow(leftLedBuffer);
        break;
      case AUTO_MODE:
        setAllSolidColor(Color.WHITE, leftLedBuffer);
        break;
      case NOTHING:
        // when the case of the robot is nothing it will be set to red
        for (int i = 0; i < leftLedBuffer.getLength(); i++) {
          leftLedBuffer.setRGB(i, Color.red.getRed(), 0, 0);
        }
        break;
      case TWENTY_SECOND_WARNING:
        setAllBlinking(Color.MAGENTA, Color.BLACK, 0.5, leftLedBuffer);

        break;
      case AMP_LINING_UP:
        setAllBlinking(Color.YELLOW, Color.BLACK, 0.5, leftLedBuffer);

        break;
    }

    leftLed.setData(leftLedBuffer);
  }

  private void setSingleStripSolidColor(
      Color color, AddressableLEDBuffer ledBuffer, int startingLED, int endingLED) {
    for (int i = startingLED; i <= endingLED; i++) {
      ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
    }
  }

  private void setAllSolidColor(Color color, AddressableLEDBuffer ledBuffer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
    }
  }

  private void setSingleStripBlinking(
      Color color1,
      Color color2,
      double seconds,
      AddressableLEDBuffer ledBuffer,
      int startingLED,
      int endingLED) {

    if (Math.sin(((2 * Math.PI) / seconds) * Timer.getFPGATimestamp()) >= 0) {
      for (int i = startingLED; i <= endingLED; i++) {
        ledBuffer.setRGB(i, color1.getRed(), color1.getGreen(), color1.getBlue());
      }
    } else if (Math.sin(((2 * Math.PI) / seconds) * Timer.getFPGATimestamp()) < 0) {
      for (int i = startingLED; i <= endingLED; i++) {
        ledBuffer.setRGB(i, color2.getRed(), color2.getGreen(), color2.getBlue());
      }
    }
  }

  private void setAllBlinking(
      Color color1, Color color2, double seconds, AddressableLEDBuffer ledBuffer) {

    if (Math.sin(((2 * Math.PI) / seconds) * Timer.getFPGATimestamp()) >= 0) {
      for (int i = 0; i < leftLedBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, color1.getRed(), color1.getGreen(), color1.getBlue());
      }
    } else if (Math.sin((2 * Math.PI) / seconds) * (Timer.getFPGATimestamp()) < 0) {
      for (int i = 0; i < leftLedBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, color2.getRed(), color2.getGreen(), color2.getBlue());
      }
    }
  }

  private void setAllBlinking(Color color1, Color color2) {

    if (Math.sin(Timer.getFPGATimestamp()) >= 0) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, color1.getRed(), color1.getGreen(), color1.getBlue());
      }
    } else if (Math.sin(Timer.getFPGATimestamp()) < 0) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, color2.getRed(), color2.getGreen(), color2.getBlue());
      }
    }
  }

  private void setAllSnake(Color color) {
    for (int i = 0; i < leftLedBuffer.getLength(); i++) {
      final var shade = (snakeShade + (i * 255 / leftLedBuffer.getLength())) % 255;
      leftLedBuffer.setRGB(
          i, shade * color.getRed(), shade * color.getGreen(), shade * color.getBlue());

      snakeShade += 3;
      snakeShade %= 255;
    }

  private void setSingleStripRainbow(int startingLED, int endingLED) {
    for (var i = startingLED; i <= endingLED; i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / leftLedBuffer.getLength())) % 180;
      leftLedBuffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  private void setAllRainbow(AddressableLEDBuffer ledBuffer) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / leftLedBuffer.getLength())) % 180;
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
    SHOOTER_SUCCESS, //
    SHOOTER_LINED_UP, //
    DRIVING_TO_GAMEPIECE, //
    AUTO_MODE, //
    NOTHING, //
    TWENTY_SECOND_WARNING, //
    AMP_LINING_UP;
  }

  public void setRobotState(robotStates robotState) {
    this.robotState = robotState;
  }
}
