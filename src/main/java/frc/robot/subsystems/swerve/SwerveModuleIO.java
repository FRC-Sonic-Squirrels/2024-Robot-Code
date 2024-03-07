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
import frc.robot.Constants;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  public class Fake implements SwerveModuleIO {
    @Override
    public void registerSignalForOdometry(List<BaseStatusSignal> signals) {}

    @Override
    public SwerveModulePosition updateOdometry(ModuleIOInputs inputs, double wheelRadius) {
      return new SwerveModulePosition();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {}

    @Override
    public void setDriveVoltage(double volts) {}

    @Override
    public void setTurnVoltage(double volts) {}

    @Override
    public void setDriveBrakeMode(boolean enable) {}

    @Override
    public void setTurnBrakeMode(boolean enable) {}
  }

  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad;
    public double driveVelocityRadPerSec;
    public double driveAppliedVolts;
    public double driveCurrentAmps;

    public Rotation2d turnAbsolutePosition = Constants.zeroRotation2d;
    public Rotation2d turnPosition = Constants.zeroRotation2d;
    public double turnVelocityRadPerSec;
    public double turnAppliedVolts;
    public double turnCurrentAmps;
  }

  void registerSignalForOdometry(List<BaseStatusSignal> signals);

  /** Updates the odometry position. */
  SwerveModulePosition updateOdometry(ModuleIOInputs inputs, double wheelRadius);

  /** Updates the set of loggable inputs. */
  void updateInputs(ModuleIOInputs inputs);

  /** Run the drive motor at the specified voltage. */
  void setDriveVoltage(double volts);

  /** Run the turn motor at the specified voltage. */
  void setTurnVoltage(double volts);

  /** Enable or disable brake mode on the drive motor. */
  void setDriveBrakeMode(boolean enable);

  /** Enable or disable brake mode on the turn motor. */
  void setTurnBrakeMode(boolean enable);
}
