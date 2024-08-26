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

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.team2930.IsaacSimDispatcher;
import frc.robot.configs.IndividualSwerveModuleConfig;
import frc.robot.configs.RobotConfig;

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

    private double lastTurnPosition = 0;

    public SwerveModuleIOIsaacSim(RobotConfig globalConfig, IndividualSwerveModuleConfig moduleSpecificConfig, IsaacSimDispatcher dispatcher){
        this.distanceToRotation =
        1.0 / globalConfig.getWheelRadius().in(edu.wpi.first.units.Units.Meters);

        this.moduleSpecificConfig = moduleSpecificConfig;
        this.dispatcher = dispatcher;

        absoluteEncoderOffset = moduleSpecificConfig.absoluteEncoderOffset();
    }

    @Override
    public void registerSignalForOdometry(List<BaseStatusSignal> signals) {}

    @Override
    public SwerveModulePosition updateOdometry(Inputs inputs, double wheelRadius) {
        loops++;
        double currentTime = RobotController.getFPGATime();
        double timeDifference = currentTime - lastUpdatedTimestamp;
        if(timeDifference >= 1.0){
            lastUpdatedTimestamp = currentTime;
            deltaTime = timeDifference/((double)(loops-lastUpdatedLoops) * 1000000.0);
            lastUpdatedLoops = loops;
        }

        drivePositionRaw += inputs.driveVelocityRadPerSec * deltaTime;
        inputs.drivePositionRad = drivePositionRaw;
        
        var angleRelative = Rotation2d.fromRadians(dispatcher.recieveMotorPos(moduleSpecificConfig.steerMotorCANID()));
        inputs.turnPosition = angleRelative;

        // Process turn encoder position.
        turnAbsolutePosition += inputs.turnVelocityRadPerSec * deltaTime;
        inputs.turnAbsolutePosition = Rotation2d.fromRadians(turnAbsolutePosition).minus(absoluteEncoderOffset);

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
        inputs.driveVelocityRadPerSec = dispatcher.recieveMotorVel(moduleSpecificConfig.driveMotorCANID());
        inputs.turnVelocityRadPerSec = dispatcher.recieveMotorVel(moduleSpecificConfig.steerMotorCANID());
    }

    @Override
    public void setDriveVelocity(
        double velocityMetersPerSec, double accelerationMetersPerSecondSquared) {
        // m/s -> divide by wheel radius to get radians/s -> convert to rotations
        var velocityRadiansPerSecond =
            velocityMetersPerSec * distanceToRotation;
    
        dispatcher.sendMotorInfo(moduleSpecificConfig.driveMotorCANID(), velocityRadiansPerSecond);
    }
  
    @Override
    public void setTurnPosition(Rotation2d position) {
        double turnPosition = position.getRadians();
        dispatcher.sendMotorInfo(moduleSpecificConfig.steerMotorCANID(), getClosestPositionRadians(turnPosition, lastTurnPosition) + absoluteEncoderOffset.getRadians());
        lastTurnPosition = turnPosition;
    }

    private double getClosestPositionRadians(double input, double lastInput){
        double attempt = 0;
        double previousAttempt = input;
        for (int i = 1; true; i++) {
            attempt = i * Math.PI * 2 * (lastInput > input ? 1.0 : -1.0);
            if(Math.abs(attempt - lastInput) > Math.abs(previousAttempt - lastInput)){
                return previousAttempt;
            }
            previousAttempt = attempt;
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