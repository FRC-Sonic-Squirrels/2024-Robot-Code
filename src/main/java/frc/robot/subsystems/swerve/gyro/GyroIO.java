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

package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Constants.zeroRotation2d;
    public double yawVelocityRadPerSec;
    public double xAcceleration;
    public double yAcceleration;
    public double zAcceleration;
  }

  public class Fake implements GyroIO {
    @Override
    public void registerSignalForOdometry(List<BaseStatusSignal> signal) {}

    @Override
    public Rotation2d updateOdometry(GyroIOInputs inputs) {
      return null;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {}
  }

  void registerSignalForOdometry(List<BaseStatusSignal> signal);

  Rotation2d updateOdometry(GyroIOInputs inputs);

  void updateInputs(GyroIOInputs inputs);
}
