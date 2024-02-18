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
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.SwerveModule;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yaw;
  private final Queue<Double> yawPositionQueue;
  private final StatusSignal<Double> yawVelocity;

  private final StatusSignal<Double> xAcceleration;
  private final StatusSignal<Double> yAcceleration;
  private final StatusSignal<Double> zAcceleration;

  public GyroIOPigeon2(RobotConfig config) {
    Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
    pigeonConfig.MountPose.MountPosePitch = config.getGyroMountingPitch();
    pigeonConfig.MountPose.MountPoseRoll = config.getGyroMountingRoll();
    pigeonConfig.MountPose.MountPoseYaw = config.getGyroMountingYaw();

    pigeon = new Pigeon2(config.getGyroCANID(), config.getCANBusName());

    pigeon.getConfigurator().apply(pigeonConfig);

    yaw = pigeon.getYaw();
    // FIXME: is this the correct method call
    yawVelocity = pigeon.getAngularVelocityZDevice();

    xAcceleration = pigeon.getAccelerationX();
    yAcceleration = pigeon.getAccelerationY();
    zAcceleration = pigeon.getAccelerationZ();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(SwerveModule.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(100.0);
    xAcceleration.setUpdateFrequency(100.0);
    yAcceleration.setUpdateFrequency(100.0);
    zAcceleration.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
    yawPositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(pigeon, pigeon.getYaw(), config.getCANBusName());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, xAcceleration, yAcceleration, zAcceleration)
            .equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Math.toRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    yawPositionQueue.clear();

    inputs.xAcceleration = xAcceleration.getValueAsDouble();
    inputs.yAcceleration = yAcceleration.getValueAsDouble();
    inputs.zAcceleration = zAcceleration.getValueAsDouble();
  }
}
