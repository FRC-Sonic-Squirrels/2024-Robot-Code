// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.IsaacSimDispatcher;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.configs.IndividualSwerveModuleConfig;
import frc.robot.configs.RobotConfig;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOIsaacSim implements SwerveModuleIO {
  private final double distanceToRotation;

  private IndividualSwerveModuleConfig moduleSpecificConfig;

  private IsaacSimDispatcher dispatcher;

  private double lastPositionMeters;
  private Rotation2d turnRelativeOffset;

  private final Rotation2d absoluteEncoderOffset;
  private double turnAbsolutePosition;
  private double drivePositionRaw;

  private int loops;
  private double lastUpdatedTimestamp = 0;
  private int lastUpdatedLoops = 0;
  private double deltaTime = 0;

  private TunableNumberGroup group = new TunableNumberGroup("turnGroup");
  private LoggedTunableNumber tunable;

  private double lastRotation;

  private boolean driveFlipped = false;

  public SwerveModuleIOIsaacSim(
      RobotConfig globalConfig,
      IndividualSwerveModuleConfig moduleSpecificConfig,
      IsaacSimDispatcher dispatcher) {
    this.distanceToRotation =
        1.0 / globalConfig.getWheelRadius().in(edu.wpi.first.units.Units.Meters);

    this.moduleSpecificConfig = moduleSpecificConfig;
    this.dispatcher = dispatcher;

    absoluteEncoderOffset = moduleSpecificConfig.absoluteEncoderOffset();
    tunable = group.build("turn" + moduleSpecificConfig.steerMotorCANID(), 0);
  }

  @Override
  public void registerSignalForOdometry(List<BaseStatusSignal> signals) {}

  @Override
  public SwerveModulePosition updateOdometry(Inputs inputs, double wheelRadius) {
    loops++;
    double currentTime = RobotController.getFPGATime();
    double timeDifference = currentTime - lastUpdatedTimestamp;
    if (timeDifference >= 1.0) {
      lastUpdatedTimestamp = currentTime;
      deltaTime = timeDifference / ((double) (loops - lastUpdatedLoops) * 1000000.0);
      lastUpdatedLoops = loops;
    }

    drivePositionRaw += inputs.driveVelocityRadPerSec * deltaTime;
    inputs.drivePositionRad = drivePositionRaw;

    var angleRelative =
        Rotation2d.fromRadians(
            GeometryUtil.optimizeRotation(
                dispatcher.recieveMotorPos(moduleSpecificConfig.steerMotorCANID())));
    Logger.recordOutput(
        "steer" + moduleSpecificConfig.steerMotorCANID() + "position", angleRelative.getRadians());
    inputs.turnPosition = angleRelative;

    // Process turn encoder position.
    turnAbsolutePosition += inputs.turnVelocityRadPerSec * deltaTime;
    inputs.turnAbsolutePosition =
        Rotation2d.fromRadians(turnAbsolutePosition).minus(absoluteEncoderOffset);

    if (DriverStation.isDisabled())
      turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);

    inputs.angle = angleRelative.plus(turnRelativeOffset);

    var positionMeters = drivePositionRaw * wheelRadius;
    var res = new SwerveModulePosition(positionMeters - lastPositionMeters, inputs.angle);
    lastPositionMeters = positionMeters;

    return res;
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.driveVelocityRadPerSec =
        dispatcher.recieveMotorVel(moduleSpecificConfig.driveMotorCANID());
    inputs.turnVelocityRadPerSec =
        dispatcher.recieveMotorVel(moduleSpecificConfig.steerMotorCANID());
  }

  @Override
  public void setDriveVelocity(
      double velocityMetersPerSec, double accelerationMetersPerSecondSquared) {
    // m/s -> divide by wheel radius to get radians/s -> convert to rotations
    var velocityRadiansPerSecond =
        velocityMetersPerSec * distanceToRotation * (driveFlipped ? -1.0 : 1.0);

    dispatcher.sendMotorInfo(moduleSpecificConfig.driveMotorCANID(), velocityRadiansPerSecond);
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    double turnPosition =
        findclosestrotation(
            position.getRadians() + absoluteEncoderOffset.getRadians(), lastRotation);
    Logger.recordOutput(
        "targetSteerPosition" + moduleSpecificConfig.steerMotorCANID(),
        GeometryUtil.optimizeRotation(turnPosition));
    Logger.recordOutput(
        "currentSteerPosition" + moduleSpecificConfig.steerMotorCANID(),
        GeometryUtil.optimizeRotation(turnPosition));
    dispatcher.sendMotorInfo(moduleSpecificConfig.steerMotorCANID(), turnPosition);
    lastRotation = turnPosition;
  }

  private double findclosestrotation(double currentTarget, double lastTarget) {
    double attempt = currentTarget;
    double lastAttempt = currentTarget;
    for (int i = 0; true; i++) {
      attempt += Math.PI * (lastTarget > currentTarget ? 1.0 : -1.0);
      if (Math.abs(lastAttempt - lastTarget) < Math.abs(attempt - lastTarget)) {
        driveFlipped = !(Math.round(currentTarget - lastAttempt) % 2 == 0);
        return lastAttempt;
      }
      lastAttempt = attempt;
    }
  }

  @Override
  public void setDriveClosedLoopConstraints(
      double kP, double kD, double kS, double kV, double kA) {}

  @Override
  public void setTurnClosedLoopConstraints(
      double kP, double kD, double cruiseVelocity, double acceleration) {}

  @Override
  public void reconfigureMotors() {}

  @Override
  public void setDriveVoltage(double volts) {}

  @Override
  public void setTurnVoltage(double volts) {}

  @Override
  public void setDriveBrakeMode(boolean enable) {}

  @Override
  public void setTurnBrakeMode(boolean enable) {}
}
