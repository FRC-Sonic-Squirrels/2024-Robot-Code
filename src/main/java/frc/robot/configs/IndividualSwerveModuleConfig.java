package frc.robot.configs;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.SwerveModuleConstants;

public record IndividualSwerveModuleConfig(
    int driveMotorCANID,
    int steerMotorCANID,
    int steerEncoderCANID,
    Rotation2d absoluteEncoderOffset,
    InvertedValue driveMotorInverted,
    InvertedValue steerMotorInverted) {

  public static IndividualSwerveModuleConfig configureWithDefaultInverts(
      int driveMotorCANID,
      int steerMotorCANID,
      int steerEncoderCANID,
      Rotation2d absoluteEncoderOffset) {
    return new IndividualSwerveModuleConfig(
        driveMotorCANID,
        steerMotorCANID,
        steerEncoderCANID,
        absoluteEncoderOffset,
        SwerveModuleConstants.MK4I.DEFAULT_DRIVE_MOTOR_INVERT,
        SwerveModuleConstants.MK4I.DEFAULT_STEER_MOTOR_INVERT);
  }
}
