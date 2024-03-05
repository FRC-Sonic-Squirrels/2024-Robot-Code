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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.configs.RobotConfig;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  public static final double ODOMETRY_FREQUENCY = 250.0;

  private final SwerveModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private SimpleMotorFeedforward driveFeedforward;
  private final PIDController driveFeedback;
  private final PIDController turnFeedback;
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset = Constants.zeroRotation2d; // Relative + Offset = Absolute
  private boolean turnRelativeOffsetInitialized; // Relative + Offset = Absolute
  private double lastPositionMeters = 0.0; // Used for delta calculation
  private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[] {};
  private double[] odometryTimestamps = new double[] {};

  private final double WHEEL_RADIUS;

  private final LoggedTunableNumber driveKS;
  private final LoggedTunableNumber driveKV;
  private final LoggedTunableNumber driveKA;

  private final LoggedTunableNumber driveKP;
  private final LoggedTunableNumber driveKD;

  private final LoggedTunableNumber angleKP;
  private final LoggedTunableNumber angleKD;

  private final RobotConfig config;

  public SwerveModule(int index, RobotConfig config, SwerveModuleIO io) {
    this.io = io;
    this.index = index;

    this.config = config;

    this.WHEEL_RADIUS = config.getWheelRadius().in(Units.Meters);

    driveKS = config.getDriveKS();
    driveKV = config.getDriveKV();
    driveKA = config.getDriveKA();

    driveKP = config.getDriveKP();
    driveKD = config.getDriveKD();

    angleKP = config.getAngleKP();
    angleKD = config.getAngleKD();

    driveFeedforward = new SimpleMotorFeedforward(driveKS.get(), driveKV.get());
    driveFeedback = new PIDController(driveKP.get(), 0.0, driveKD.get());
    turnFeedback = new PIDController(angleKP.get(), 0.0, angleKD.get());

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
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
    try (var ignored = new ExecutionTiming("SwerveModule" + Integer.toString(index))) {
      Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

      // update Tuneable PID and FF values
      int hc = hashCode();
      if (driveKS.hasChanged(hc)
          || driveKV.hasChanged(hc)
          || driveKA.hasChanged(hc)
          || driveKP.hasChanged(hc)
          || driveKD.hasChanged(hc)
          || angleKP.hasChanged(hc)
          || angleKD.hasChanged(hc)) {
        driveFeedforward = new SimpleMotorFeedforward(driveKS.get(), driveKV.get());

        driveFeedback.setPID(driveKP.get(), 0.0, driveKD.get());
        turnFeedback.setPID(angleKP.get(), 0.0, angleKD.get());
      }

      // On first cycle, reset relative turn encoder
      // Wait until absolute angle is nonzero in case it wasn't initialized yet
      if (!turnRelativeOffsetInitialized && inputs.turnAbsolutePosition.getRadians() != 0.0) {
        turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
        turnRelativeOffsetInitialized = true;
      }

      // Run closed loop turn control
      if (angleSetpoint != null) {
        io.setTurnVoltage(
            turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

        // Run closed loop drive control
        // Only allowed if closed loop turn control is running
        if (speedSetpoint != null) {
          // Scale velocity based on turn error
          //
          // When the error is 90^, the velocity setpoint should be 0. As the wheel turns
          // towards the setpoint, its velocity should increase. This is achieved by
          // taking the component of the velocity in the direction of the setpoint.
          double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

          // Run drive controller
          double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
          io.setDriveVoltage(
              driveFeedforward.calculate(velocityRadPerSec)
                  + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
        }
      }

      // Calculate position deltas for odometry
      int deltaCount =
          Math.min(inputs.odometryDrivePositionsRad.length, inputs.odometryTurnPositions.length);
      positionDeltas = new SwerveModulePosition[deltaCount];
      for (int i = 0; i < deltaCount; i++) {
        double positionMeters = inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS;
        Rotation2d angle = inputs.odometryTurnPositions[i].plus(turnRelativeOffset);
        positionDeltas[i] = new SwerveModulePosition(positionMeters - lastPositionMeters, angle);
        lastPositionMeters = positionMeters;

        odometryTimestamps = new double[deltaCount];
        System.arraycopy(inputs.odometryTimestamps, 0, odometryTimestamps, 0, deltaCount);
      }
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = Constants.zeroRotation2d;

    // Open loop drive control
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
    return turnRelativeOffsetInitialized
        ? inputs.turnPosition.plus(turnRelativeOffset)
        : Constants.zeroRotation2d;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * WHEEL_RADIUS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position deltas received this cycle. */
  public SwerveModulePosition[] getPositionDeltas() {
    return positionDeltas;
  }

  public double[] getOdometryTimestamps() {
    return odometryTimestamps;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  public void getCurrentAmps(double[] array, int offset) {
    array[offset] = inputs.driveCurrentAmps;
    array[offset + 1] = inputs.turnCurrentAmps;
  }
}
