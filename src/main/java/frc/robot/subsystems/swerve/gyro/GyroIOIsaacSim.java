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
import frc.lib.team2930.IsaacSimDispatcher;
import frc.robot.configs.RobotConfig;
import java.util.List;

/** IO implementation for IsaacSim */
public class GyroIOIsaacSim implements GyroIO {

  private IsaacSimDispatcher dispatcher;

  public GyroIOIsaacSim(RobotConfig config, IsaacSimDispatcher dispatcher) {
    this.dispatcher = dispatcher;
  }

  @Override
  public void registerSignalForOdometry(List<BaseStatusSignal> signals) {}

  @Override
  public Rotation2d updateOdometry(Inputs inputs) {
    var gyroRotation = Rotation2d.fromRadians(dispatcher.recieveSensorInfo("imu"));
    inputs.yawPosition = gyroRotation;
    return gyroRotation;
  }

  @Override
  public void updateInputs(Inputs inputs) {

    inputs.statusCode = "Okay";
    inputs.connected = true;
    inputs.yawVelocityRadPerSec = 0;
    inputs.xAcceleration = 0;
    inputs.yAcceleration = 0;
    inputs.zAcceleration = 0;
    inputs.statusCode = "";
    inputs.description = "";
  }

  @Override
  public void reconfigurePigeon() {
  }
}
