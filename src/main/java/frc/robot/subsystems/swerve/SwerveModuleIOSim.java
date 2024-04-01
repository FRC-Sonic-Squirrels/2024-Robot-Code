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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.configs.RobotConfig2024;
import java.util.List;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  private static final double LOOP_PERIOD_SECS = Constants.kDefaultPeriod;

  private final DCMotorSim driveSim =
      new DCMotorSim(DCMotor.getKrakenX60(1), RobotConfig2024.SWERVE_DRIVE_GEAR_RATIO, 0.025);
  private final DCMotorSim turnSim =
      new DCMotorSim(DCMotor.getFalcon500(1), RobotConfig2024.SWERVE_STEER_GEAR_RATIO, 0.004);

  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController driveFeedback;
  private final PIDController turnFeedback;
  private final double wheelRadius;

  private double lastPositionMeters;
  private double driveAppliedVolts;
  private double turnAppliedVolts;

  private double speedSetpoint;
  private Rotation2d angleSetpoint;

  public SwerveModuleIOSim() {
    driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
    driveFeedback = new PIDController(0.1, 0.0, 0.0);
    turnFeedback = new PIDController(10.0, 0.0, 0);

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);

    wheelRadius = Units.Inches.of(2.0).in(Units.Meters);
  }

  @Override
  public void registerSignalForOdometry(List<BaseStatusSignal> signals) {}

  @Override
  public SwerveModulePosition updateOdometry(Inputs inputs, double wheelRadius) {
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    double positionMeters = inputs.drivePositionRad * wheelRadius;

    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());

    var angleRelative = inputs.turnPosition;
    var res = new SwerveModulePosition(positionMeters - lastPositionMeters, angleRelative);
    lastPositionMeters = positionMeters;

    inputs.angle = angleRelative;

    return res;
  }

  @Override
  public void updateInputs(Inputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    // Run closed loop turn control
    if (angleSetpoint != null) {
      setTurnVoltage(
          turnFeedback.calculate(inputs.turnPosition.getRadians(), angleSetpoint.getRadians()));
    }

    double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

    // Run drive controller
    double velocityRadPerSec = adjustSpeedSetpoint / wheelRadius;
    setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
    speedSetpoint = 0;
    angleSetpoint = null;
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {}

  @Override
  public void setTurnBrakeMode(boolean enable) {}

  @Override
  public void setDriveVelocity(
      double velocityMetersPerSec, double accelerationMetersPerSecondSquared) {
    speedSetpoint = velocityMetersPerSec;
  }

  @Override
  public void setDriveClosedLoopConstraints(
      double kP, double kD, double kS, double kV, double kA) {}

  @Override
  public void setTurnPosition(Rotation2d position) {
    angleSetpoint = position;
  }

  @Override
  public void setTurnClosedLoopConstraints(
      double kP, double kD, double cruiseVelocity, double acceleration) {}

  @Override
  public void setNeutralMode(NeutralModeValue mode) {}
}
