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

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoublePublisher;
import frc.lib.team2930.IsaacSimDispatcher;
import frc.robot.RobotContainer;
import frc.robot.configs.IndividualSwerveModuleConfig;
import frc.robot.configs.RobotConfig;

public class SwerveModuleIOIsaacSim implements SwerveModuleIO {
    private final double distanceToRotation;

    private IndividualSwerveModuleConfig moduleSpecificConfig;

    private IsaacSimDispatcher dispatcher;

    public SwerveModuleIOIsaacSim(RobotConfig globalConfig, IndividualSwerveModuleConfig moduleSpecificConfig, IsaacSimDispatcher dispatcher){
        this.distanceToRotation =
        1.0 / globalConfig.getWheelRadius().in(edu.wpi.first.units.Units.Meters);

        this.moduleSpecificConfig = moduleSpecificConfig;
        this.dispatcher = dispatcher;
    }

    @Override
    public void registerSignalForOdometry(List<BaseStatusSignal> signals) {}

    @Override
    public SwerveModulePosition updateOdometry(Inputs inputs, double wheelRadius) {
        return new SwerveModulePosition(0, new Rotation2d());
    }

    @Override
    public void updateInputs(Inputs inputs) {}

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
        dispatcher.sendMotorInfo(moduleSpecificConfig.steerMotorCANID(), position.getRadians() + moduleSpecificConfig.absoluteEncoderOffset().getRadians());
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