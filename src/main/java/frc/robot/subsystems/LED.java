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
  AddressableLED leftLed1 = new AddressableLED(Constants.LEDConstants.PWM_PORT_1);

  AddressableLED leftLed2 = new AddressableLED(Constants.LEDConstants.PWM_PORT_2);
  AddressableLED rightLed1 = new AddressableLED(Constants.LEDConstants.PWM_PORT_3);
  AddressableLED rightLed2 = new AddressableLED(Constants.LEDConstants.PWM_PORT_4);

  int ledLength = 60;

  AddressableLEDBuffer leftLedBuffer1 = new AddressableLEDBuffer(ledLength);
  AddressableLEDBuffer leftLedBuffer2 = new AddressableLEDBuffer(ledLength);
  AddressableLEDBuffer rightLedBuffer1 = new AddressableLEDBuffer(ledLength);
  AddressableLEDBuffer rightLedBuffer2 = new AddressableLEDBuffer(ledLength);

  int snakeShade = 0;
  int rainbowFirstPixelHue = 0;
  robotStates robotState = robotStates.NOTHING;

  public LED() {
    leftLed1.setLength(leftLedBuffer1.getLength());
    leftLed1.setData(leftLedBuffer1);
    leftLed1.start();

    leftLed2.setLength(leftLedBuffer2.getLength());
    leftLed2.setData(leftLedBuffer2);
    leftLed2.start();

    rightLed1.setLength(rightLedBuffer1.getLength());
    rightLed1.setData(rightLedBuffer1);
    rightLed1.start();

    rightLed2.setLength(rightLedBuffer2.getLength());
    rightLed2.setData(rightLedBuffer2);
    rightLed2.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (robotState) {
      case SHOOTER_SUCCESS:
        // test writing solid color
        // FIXME: if wanted, inside of setallsolidcolor could remove parameters once we have certain
        // values we want to use
        setAllSolidColor(Color.RED, leftLedBuffer1);
        break;
      case SHOOTER_LINED_UP:
        // test of writing blinking
        // FIXME: if wanted, inside of setallblinking could remove paramters once we have certain
        // values we want to use
        setAllBlinking(Color.BLACK, Color.WHITE, 0.5, leftLedBuffer1);
        break;
      case DRIVING_TO_GAMEPIECE:
        setAllRainbow(leftLedBuffer1);
        break;
      case AUTO_MODE:
        setAllSolidColor(Color.WHITE, leftLedBuffer1);
        break;
      case NOTHING:
        // when the case of the robot is nothing it will be set to red
        for (int i = 0; i < leftLedBuffer1.getLength(); i++) {
          leftLedBuffer1.setRGB(i, Color.red.getRed(), 0, 0);
        }
        break;
      case TWENTY_SECOND_WARNING:
        setAllBlinking(Color.MAGENTA, Color.BLACK, 0.5, leftLedBuffer1);

        break;
      case AMP_LINING_UP:
        setAllBlinking(Color.YELLOW, Color.BLACK, 0.5, leftLedBuffer1);
        break;
      case TEST:
        setAllSnake(Color.GREEN);
    }

    leftLed1.setData(leftLedBuffer1);
    leftLed2.setData(rightLedBuffer1);
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
      for (int i = 0; i < leftLedBuffer1.getLength(); i++) {
        ledBuffer.setRGB(i, color1.getRed(), color1.getGreen(), color1.getBlue());
      }
    } else if (Math.sin((2 * Math.PI) / seconds) * (Timer.getFPGATimestamp()) < 0) {
      for (int i = 0; i < leftLedBuffer1.getLength(); i++) {
        ledBuffer.setRGB(i, color2.getRed(), color2.getGreen(), color2.getBlue());
      }
    }
  }

  private void setAllSnake(Color color) {
    for (int i = 0; i < leftLedBuffer1.getLength(); i++) {
      final var shade = (snakeShade + (i * 255 / leftLedBuffer1.getLength())) % 255;
      leftLedBuffer1.setRGB(
          i, shade * color.getRed(), shade * color.getGreen(), shade * color.getBlue());

      snakeShade += 3;
      snakeShade %= 255;
    }
  }

  private void setSingleStripRainbow(int startingLED, int endingLED) {
    for (var i = startingLED; i <= endingLED; i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / leftLedBuffer1.getLength())) % 180;
      leftLedBuffer1.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  private void setAllRainbow(AddressableLEDBuffer ledBuffer) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / leftLedBuffer1.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  private void setNothing(AddressableLEDBuffer ledBuffer) {
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
    AMP_LINING_UP,
    TEST;
  }

  public void setRobotState(robotStates robotState) {
    this.robotState = robotState;
  }
}
