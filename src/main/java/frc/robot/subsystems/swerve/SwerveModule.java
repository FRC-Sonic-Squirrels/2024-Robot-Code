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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.configs.RobotConfig;
import java.util.List;

public class SwerveModule {
  public static final int ODOMETRY_FREQUENCY = 250;

  public static final TunableNumberGroup group = new TunableNumberGroup("RobotConfig");
  public static final LoggedTunableNumber turnCruiseVelocity =
      group.build("SwerveTurnCruiseVelocity", 200);
  public static final LoggedTunableNumber turnAcceleration =
      group.build("SwerveTurnAcceleration", 200);

  private final LoggerEntry.Decimal log_drivePositionRad;
  private final LoggerEntry.Decimal log_driveVelocityRadPerSec;
  private final LoggerEntry.Decimal log_driveAppliedVolts;
  private final LoggerEntry.Decimal log_driveCurrentAmps;
  private final LoggerEntry.Decimal log_turnAbsolutePosition;
  private final LoggerEntry.Decimal log_turnPosition;
  private final LoggerEntry.Decimal log_turnVelocityRadPerSec;
  private final LoggerEntry.Decimal log_turnAppliedVolts;
  private final LoggerEntry.Decimal log_turnCurrentAmps;
  private final LoggerEntry.Decimal log_angle;
  private final ExecutionTiming timing;

  private final SwerveModuleIO io;
  private final SwerveModuleIO.Inputs inputs = new SwerveModuleIO.Inputs();
  private final double wheelRadius;

  private final LoggedTunableNumber driveKS;
  private final LoggedTunableNumber driveKV;
  private final LoggedTunableNumber driveKA;

  private final LoggedTunableNumber driveKP;
  private final LoggedTunableNumber driveKD;

  private final LoggedTunableNumber angleKP;
  private final LoggedTunableNumber angleKD;

  private Rotation2d angleSetpoint; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint; // Setpoint for closed loop control, null for open loop
  private double driveMotorMotionMagicAcceleration;

  public SwerveModule(int index, RobotConfig config, SwerveModuleIO io) {
    this.io = io;

    String key = "SwerveModule" + index;
    var group = LoggerGroup.build(key);
    log_drivePositionRad = group.buildDecimal("DrivePositionRad");
    log_driveVelocityRadPerSec = group.buildDecimal("DriveVelocityRadPerSec");
    log_driveAppliedVolts = group.buildDecimal("DriveAppliedVolts");
    log_driveCurrentAmps = group.buildDecimal("DriveCurrentAmps");
    log_turnAbsolutePosition = group.buildDecimal("TurnAbsolutePosition");
    log_turnPosition = group.buildDecimal("TurnPosition");
    log_turnVelocityRadPerSec = group.buildDecimal("TurnVelocityRadPerSec");
    log_turnAppliedVolts = group.buildDecimal("TurnAppliedVolts");
    log_turnCurrentAmps = group.buildDecimal("TurnCurrentAmps");
    log_angle = group.buildDecimal("Angle");
    timing = new ExecutionTiming(key);

    wheelRadius = config.getWheelRadius().in(Units.Meters);

    driveKS = config.getDriveKS();
    driveKV = config.getDriveKV();
    driveKA = config.getDriveKA();

    driveKP = config.getDriveKP();
    driveKD = config.getDriveKD();

    angleKP = config.getAngleKP();
    angleKD = config.getAngleKD();

    updateConstants();

    setBrakeMode(true);
  }

  public void registerSignalForOdometry(List<BaseStatusSignal> signals) {
    io.registerSignalForOdometry(signals);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    // Ideally nothing else lives here and is put in periodic() instead
    io.updateInputs(inputs);
  }

  public void periodic() {
    try (var ignored = timing.start()) {
      log_drivePositionRad.info(inputs.drivePositionRad);
      log_driveVelocityRadPerSec.info(inputs.driveVelocityRadPerSec);
      log_driveAppliedVolts.info(inputs.driveAppliedVolts);
      log_driveCurrentAmps.info(inputs.driveCurrentAmps);
      log_turnAbsolutePosition.info(inputs.turnAbsolutePosition);
      log_turnPosition.info(inputs.turnPosition);
      log_turnVelocityRadPerSec.info(inputs.turnVelocityRadPerSec);
      log_turnAppliedVolts.info(inputs.turnAppliedVolts);
      log_turnCurrentAmps.info(inputs.turnCurrentAmps);
      log_angle.info(inputs.angle);

      // update Tuneable PID and FF values
      int hc = hashCode();
      if (driveKS.hasChanged(hc)
          || driveKV.hasChanged(hc)
          || driveKA.hasChanged(hc)
          || driveKP.hasChanged(hc)
          || driveKD.hasChanged(hc)
          || angleKP.hasChanged(hc)
          || angleKD.hasChanged(hc)
          || turnCruiseVelocity.hasChanged(hc)
          || turnAcceleration.hasChanged(hc)) {
        updateConstants();
      }

      // Run closed loop turn control
      if (angleSetpoint != null) {
        io.setTurnPosition(angleSetpoint);
      }

      if (speedSetpoint != null) {
        io.setDriveVelocity(speedSetpoint, driveMotorMotionMagicAcceleration);
      }
    }
  }

  private void updateConstants() {
    io.setDriveClosedLoopConstraints(
        driveKP.get(),
        driveKD.get(),
        driveKS.get(),
        driveKV.get(),
        driveKA.get()); // FIXME: do we need need ks?

    io.setTurnClosedLoopConstraints(
        angleKP.get(), angleKD.get(), turnCruiseVelocity.get(), turnAcceleration.get());
  }

  public SwerveModulePosition updateOdometry() {
    return io.updateOdometry(inputs, wheelRadius);
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(
      SwerveModuleState state, double driveMotorMotionMagicAcceleration) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    this.driveMotorMotionMagicAcceleration = driveMotorMotionMagicAcceleration;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.angle;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  public void getCurrentAmps(double[] array, int offset) {
    array[offset] = inputs.driveCurrentAmps;
    array[offset + 1] = inputs.turnCurrentAmps;
  }

  public double getAppliedVoltage() {
    return inputs.driveAppliedVolts;
  }

  public void setNeutralMode(NeutralModeValue mode) {
    io.setNeutralMode(mode);
  }
}
