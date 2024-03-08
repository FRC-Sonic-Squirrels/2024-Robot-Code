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
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import java.util.List;

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
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;
  private final double distanceToRotation;

  private VoltageOut driveVoltageRequest;
  private VoltageOut steerVoltageRequest;

  private MotionMagicVelocityVoltage driveMotionMagicVelocityRequest =
      new MotionMagicVelocityVoltage(0.0).withAcceleration(0.0).withEnableFOC(true);

  private MotionMagicVoltage turnMotionMagicVoltageRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final IndividualSwerveModuleConfig moduleSpecificConfig;

  private Rotation2d turnRelativeOffset; // Relative + Offset = Absolute
  private double lastPositionMeters;

  private final BaseStatusSignal[] refreshSet;

  public SwerveModuleIOTalonFX(
      RobotConfig globalConfig, IndividualSwerveModuleConfig moduleSpecificConfig) {

    // --- define constants ---
    this.moduleSpecificConfig = moduleSpecificConfig;

    this.distanceToRotation =
        1.0 / globalConfig.getWheelRadius().in(edu.wpi.first.units.Units.Meters);

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

    driveTalonConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

    driveTalon.getConfigurator().apply(driveTalonConfig);

    setDriveBrakeMode(true);

    // -- STEER
    var turnTalonConfig = new TalonFXConfiguration();
    turnTalonConfig.CurrentLimits = steerMotorCurrentLimitConfig;
    turnTalonConfig.MotorOutput.Inverted = steerMotorInverted;
    // turnTalonConfig.Voltage.SupplyVoltageTimeConstant = 0.02;

    turnTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnTalonConfig.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;

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
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

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
    cancoder.optimizeBusUtilization();

    refreshSet =
        new BaseStatusSignal[] {
          driveVelocity,
          driveAppliedVolts,
          driveCurrent,
          turnVelocity,
          turnAppliedVolts,
          turnCurrent
        };
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(refreshSet);

    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
  }

  @Override
  public void registerSignalForOdometry(List<BaseStatusSignal> signals) {
    signals.add(drivePosition);
    signals.add(turnPosition);
    signals.add(turnAbsolutePosition);
  }

  @Override
  public SwerveModulePosition updateOdometry(ModuleIOInputs inputs, double wheelRadius) {
    // Process drive motor position.
    var drivePositionRotations = drivePosition.getValueAsDouble();
    var drivePositionRaw = Units.rotationsToRadians(drivePositionRotations);
    inputs.drivePositionRad = drivePositionRaw;

    // Process turn motor position.
    var turnPositionAngle = turnPosition.getValueAsDouble();
    var angleRelative = Rotation2d.fromRotations(turnPositionAngle);
    inputs.turnPosition = angleRelative;

    // Process turn encoder position.
    var turnAbsolutePositionRaw = turnAbsolutePosition.getValueAsDouble();
    var turnAbsolutePositionRotations = Rotation2d.fromRotations(turnAbsolutePositionRaw);
    inputs.turnAbsolutePosition = turnAbsolutePositionRotations.minus(absoluteEncoderOffset);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (turnRelativeOffset == null) {
      if (inputs.turnAbsolutePosition.getRadians() == 0.0) {
        return null;
      }

      turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
    }

    inputs.angle = angleRelative.plus(turnRelativeOffset);

    var positionMeters = drivePositionRaw * wheelRadius;
    var res = new SwerveModulePosition(positionMeters - lastPositionMeters, inputs.angle);
    lastPositionMeters = positionMeters;

    return res;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(driveVoltageRequest.withOutput(volts));
  }

  @Override
  public void setDriveVelocity(
      double velocityMetersPerSec, double accelerationMetersPerSecondSquared) {

    // m/s -> divide by wheel radius to get radians/s -> convert to rotations
    var velocityRotationsPerSecond =
        Units.radiansToRotations(velocityMetersPerSec * distanceToRotation);
    var accelerationRotationsPerSecondSquared =
        Units.radiansToRotations(accelerationMetersPerSecondSquared * distanceToRotation);

    driveTalon.setControl(
        driveMotionMagicVelocityRequest
            .withVelocity(velocityRotationsPerSecond)
            .withAcceleration(accelerationRotationsPerSecondSquared));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(steerVoltageRequest.withOutput(volts));
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    if (turnRelativeOffset != null) {
      position = position.minus(turnRelativeOffset);
    }

    turnTalon.setControl(turnMotionMagicVoltageRequest.withPosition(position.getRotations()));
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

  @Override
  public void setDriveClosedLoopConstraints(double kP, double kD, double kS, double kV, double kA) {
    Slot0Configs pidConfig = new Slot0Configs();

    var configurator = driveTalon.getConfigurator();
    configurator.refresh(pidConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kS = kS;
    pidConfig.kV = kV;
    pidConfig.kA = kA;

    configurator.apply(pidConfig);
  }

  @Override
  public void setTurnClosedLoopConstraints(
      double kP, double kD, double cruiseVelocity, double acceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var configurator = turnTalon.getConfigurator();
    configurator.refresh(pidConfig);
    configurator.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;

    mmConfig.MotionMagicCruiseVelocity = cruiseVelocity;
    mmConfig.MotionMagicAcceleration = acceleration;

    configurator.apply(pidConfig);
    configurator.apply(mmConfig);
  }
}
