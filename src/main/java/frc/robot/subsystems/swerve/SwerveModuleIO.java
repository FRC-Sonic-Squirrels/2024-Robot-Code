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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import java.util.List;

public interface SwerveModuleIO {
  public class Fake implements SwerveModuleIO {
    @Override
    public void registerSignalForOdometry(List<BaseStatusSignal> signals) {}

    @Override
    public SwerveModulePosition updateOdometry(Inputs inputs, double wheelRadius) {
      return new SwerveModulePosition();
    }

    @Override
    public void updateInputs(Inputs inputs) {}

    @Override
    public void setDriveVoltage(double volts) {}

    @Override
    public void setTurnVoltage(double volts) {}

    @Override
    public void setDriveBrakeMode(boolean enable) {}

    @Override
    public void setTurnBrakeMode(boolean enable) {}

    @Override
    public void setDriveVelocity(
        double velocityMetersPerSec, double accelerationMetersPerSecondSquared) {}

    @Override
    public void setDriveClosedLoopConstraints(
        double kP, double kD, double kS, double kV, double kA) {}

    @Override
    public void setTurnPosition(Rotation2d position) {}

    @Override
    public void setTurnClosedLoopConstraints(
        double kP, double kD, double cruiseVelocity, double acceleration) {}

    @Override
    public void setNeutralMode(NeutralModeValue mode) {}
  }

  public static class Inputs {
    public double drivePositionRad;
    public double driveVelocityRadPerSec;
    public double driveAppliedVolts;
    public double driveCurrentAmps;

    public Rotation2d turnAbsolutePosition = Constants.zeroRotation2d;
    public Rotation2d turnPosition = Constants.zeroRotation2d;
    public double turnVelocityRadPerSec;
    public double turnAppliedVolts;
    public double turnCurrentAmps;

    public Rotation2d angle = Constants.zeroRotation2d;
  }

  void registerSignalForOdometry(List<BaseStatusSignal> signals);

  /** Updates the odometry position. */
  SwerveModulePosition updateOdometry(Inputs inputs, double wheelRadius);

  /** Updates the set of loggable inputs. */
  void updateInputs(Inputs inputs);

  /** Run the drive motor at the specified voltage. */
  void setDriveVoltage(double volts);

  void setDriveVelocity(double velocityMetersPerSec, double accelerationMetersPerSecondSquared);

  void setDriveClosedLoopConstraints(double kP, double kD, double kS, double kV, double Ka);

  /** Run the turn motor at the specified voltage. */
  void setTurnVoltage(double volts);

  void setTurnPosition(Rotation2d position);

  void setTurnClosedLoopConstraints(
      double kP, double kD, double cruiseVelocity, double acceleration);

  /** Enable or disable brake mode on the drive motor. */
  void setDriveBrakeMode(boolean enable);

  /** Enable or disable brake mode on the turn motor. */
  void setTurnBrakeMode(boolean enable);

  void setNeutralMode(NeutralModeValue mode);
}
