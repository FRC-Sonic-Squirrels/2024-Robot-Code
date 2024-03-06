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
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.robot.configs.IndividualSwerveModuleConfig;
import frc.robot.configs.RobotConfig;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {

  // -- CONSTANTS
  private final int driveMotorCANID;
  private final int steerMotorCANID;
  private final int cancoderCANID;

  private final InvertedValue driveMotorInverted;
  private final InvertedValue steerMotorInverted;

  private final Rotation2d absoluteEncoderOffset;

  private final CurrentLimitsConfigs driveMotorCurrentLimitConfig;
  private final CurrentLimitsConfigs steerMotorCurrentLimitConfig;

  private final double DRIVE_GEAR_RATIO;
  private final double TURN_GEAR_RATIO;
  // -- CONSTANTS

  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  private final Queue<Double> odometryTimestampsQueue;

  private VoltageOut driveVoltageRequest;
  private VoltageOut steerVoltageRequest;

  private final RobotConfig globalConfig;
  private final IndividualSwerveModuleConfig moduleSpecificConfig;

  private Rotation2d turnRelativeOffset; // Relative + Offset = Absolute
  private double lastPositionMeters;

  public SwerveModuleIOTalonFX(
      RobotConfig globalConfig, IndividualSwerveModuleConfig moduleSpecificConfig) {

    // --- define constants ---
    this.globalConfig = globalConfig;
    this.moduleSpecificConfig = moduleSpecificConfig;

    driveMotorCANID = moduleSpecificConfig.driveMotorCANID();
    steerMotorCANID = moduleSpecificConfig.steerMotorCANID();
    cancoderCANID = moduleSpecificConfig.steerEncoderCANID();

    driveMotorInverted = moduleSpecificConfig.driveMotorInverted();
    steerMotorInverted = moduleSpecificConfig.steerMotorInverted();

    absoluteEncoderOffset = moduleSpecificConfig.absoluteEncoderOffset();

    DRIVE_GEAR_RATIO = globalConfig.getSwerveModuleDriveGearRatio();
    TURN_GEAR_RATIO = globalConfig.getSwerveModuleTurnGearRatio();

    // driveMotorCurrentLimitConfig = globalConfig.getDriveTalonCurrentLimitConfig();
    driveMotorCurrentLimitConfig =
        new CurrentLimitsConfigs().withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);
    steerMotorCurrentLimitConfig = globalConfig.getSteerTalonCurrentLimitConfig();
    // --- define constants ---

    driveTalon = new TalonFX(driveMotorCANID, globalConfig.getCANBusName());
    turnTalon = new TalonFX(steerMotorCANID, globalConfig.getCANBusName());
    cancoder = new CANcoder(cancoderCANID, globalConfig.getCANBusName());

    // -- DRIVE
    var driveTalonConfig = new TalonFXConfiguration();
    driveTalonConfig.CurrentLimits = driveMotorCurrentLimitConfig;
    driveTalonConfig.MotorOutput.Inverted = driveMotorInverted;
    driveTalonConfig.Voltage.SupplyVoltageTimeConstant = 0.02;
    driveTalon.getConfigurator().apply(driveTalonConfig);

    setDriveBrakeMode(true);

    // -- STEER
    var turnTalonConfig = new TalonFXConfiguration();
    turnTalonConfig.CurrentLimits = steerMotorCurrentLimitConfig;
    turnTalonConfig.MotorOutput.Inverted = steerMotorInverted;
    // turnTalonConfig.Voltage.SupplyVoltageTimeConstant = 0.02;
    turnTalon.getConfigurator().apply(turnTalonConfig);

    setTurnBrakeMode(true);

    // -- CANCODER
    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    // -- configure voltage requests
    driveVoltageRequest = new VoltageOut(0.0);
    driveVoltageRequest.EnableFOC = globalConfig.getPhoenix6Licensed();

    steerVoltageRequest = new VoltageOut(0.0);
    steerVoltageRequest.EnableFOC = globalConfig.getPhoenix6Licensed();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(driveTalon, driveTalon.getPosition(), globalConfig.getCANBusName());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(turnTalon, turnPosition, globalConfig.getCANBusName());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    odometryTimestampsQueue = PhoenixOdometryThread.getInstance().hookForTimestamps();

    BaseStatusSignal.setUpdateFrequencyForAll(
        SwerveModule.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);

    inputs.odometryTimestamps =
        odometryTimestampsQueue.stream().mapToDouble((Double value) -> value).toArray();

    odometryTimestampsQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public SwerveModulePosition updateOdometry(ModuleIOInputs inputs, double wheelRadius) {
    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (turnRelativeOffset == null) {
      if (BaseStatusSignal.waitForAll(0.02, turnAbsolutePosition) != StatusCode.OK) {
        return null;
      }

      var turnAbsolutePositionRotations = turnAbsolutePosition.getValueAsDouble();
      var turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePositionRotations);
      inputs.turnAbsolutePosition = turnAbsolutePosition.minus(absoluteEncoderOffset);

      turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
    }

    var drivePositionRotations = drivePosition.getValueAsDouble() / DRIVE_GEAR_RATIO;
    var drivePositionRaw = Units.rotationsToRadians(drivePositionRotations);
    inputs.drivePositionRad = drivePositionRaw;

    var turnPositionAngle = turnPosition.getValueAsDouble() / TURN_GEAR_RATIO;
    var angleRelative = Rotation2d.fromRotations(turnPositionAngle);
    var angle = angleRelative.plus(turnRelativeOffset);

    var positionMeters = drivePositionRaw * wheelRadius;
    var res = new SwerveModulePosition(positionMeters - lastPositionMeters, angle);
    lastPositionMeters = positionMeters;

    return res;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(driveVoltageRequest.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(steerVoltageRequest.withOutput(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    driveTalon.getConfigurator().refresh(config);

    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    turnTalon.getConfigurator().refresh(config);

    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
