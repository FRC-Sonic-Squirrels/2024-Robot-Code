// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;

public class Arm extends SubsystemBase {
  private static final String ROOT_TABLE = "Arm";

  private static final LoggerEntry logInputs = new LoggerEntry(ROOT_TABLE);
  private static final LoggerGroup logGroup = new LoggerGroup(ROOT_TABLE);
  private static final LoggerEntry logPositionDegrees = logGroup.build("PositionDegrees");
  private static final LoggerEntry logControlMode = logGroup.build("ControlMode");
  private static final LoggerEntry logTargetClosedLoopSetpointDegrees =
      logGroup.build("targetClosedLoopSetpointDegrees");

  public static final LoggerEntry logSIM_FF_fG = logGroup.build("SIM_FF_fG");
  public static final LoggerEntry logSIM_error = logGroup.build("SIM_error");
  public static final LoggerEntry logSIM_controlEffort = logGroup.build("SIM_controlEffort");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kD = group.build("kD");
  private static final LoggedTunableNumber kG = group.build("kG");

  private static final LoggedTunableNumber closedLoopMaxVelocityConstraint =
      group.build("defaultClosedLoopMaxVelocityConstraint");
  private static final LoggedTunableNumber closedLoopMaxAccelerationConstraint =
      group.build("defaultClosedLoopMaxAccelerationConstraint");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_MAESTRO) {
      kP.initDefault(70.0);
      kD.initDefault(1.6);
      kG.initDefault(0.0);

      // FIXME: find the theoritical from the JVN docs
      closedLoopMaxVelocityConstraint.initDefault(80);
      closedLoopMaxAccelerationConstraint.initDefault(160);
    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {

      kP.initDefault(2.5);
      kD.initDefault(0);
      kG.initDefault(0.0);

      closedLoopMaxVelocityConstraint.initDefault(40);
      closedLoopMaxAccelerationConstraint.initDefault(80);
    }
  }

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private ControlMode currentControlMode = ControlMode.OPEN_LOOP;
  private Rotation2d closedLoopTargetAngle = Constants.zeroRotation2d;
  // FIXME: tune this value
  private final Rotation2d closedLoopTolerance = Rotation2d.fromDegrees(1);
  private static final double MAX_VOLTAGE = 12;

  /** Creates a new ArmSubsystem. */
  public Arm(ArmIO io) {
    this.io = io;

    // io.resetSensorPosition(Constants.ArmConstants.HOME_POSITION);
    io.setVoltage(0.0);

    io.setClosedLoopConstants(
        kP.get(),
        kD.get(),
        kG.get(),
        closedLoopMaxVelocityConstraint.get(),
        closedLoopMaxAccelerationConstraint.get());
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming("Arm")) {
      io.updateInputs(inputs);
      logInputs.info(inputs);

      logPositionDegrees.info(inputs.armPosition.getDegrees() % 360);
      logControlMode.info(currentControlMode);
      logTargetClosedLoopSetpointDegrees.info(closedLoopTargetAngle.getDegrees());

      // ---- UPDATE TUNABLE NUMBERS
      var hc = hashCode();
      if (kP.hasChanged(hc)
          || kD.hasChanged(hc)
          || kG.hasChanged(hc)
          || closedLoopMaxVelocityConstraint.hasChanged(hc)
          || closedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        io.setClosedLoopConstants(
            kP.get(),
            kD.get(),
            kG.get(),
            closedLoopMaxVelocityConstraint.get(),
            closedLoopMaxAccelerationConstraint.get());
      }
    }
  }

  public void setAngle(Rotation2d angle) {
    angle =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                angle.getRadians(),
                Constants.ArmConstants.MIN_ARM_ANGLE.getRadians(),
                Constants.ArmConstants.MAX_ARM_ANGLE.getRadians()));

    currentControlMode = ControlMode.CLOSED_LOOP;
    closedLoopTargetAngle = angle;
    io.setClosedLoopPosition(angle);
  }

  public void resetSubsystem() {
    currentControlMode = ControlMode.OPEN_LOOP;
    io.setVoltage(0);
  }

  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -MAX_VOLTAGE, MAX_VOLTAGE);
    currentControlMode = ControlMode.OPEN_LOOP;
    io.setVoltage(volts);
  }

  public Rotation2d getAngle() {
    return inputs.armPosition;
  }

  public void resetSensorToHomePosition() {
    io.resetSensorPosition(Constants.ArmConstants.MIN_ARM_ANGLE);
  }

  public boolean isAtTargetAngle() {
    return isAtTargetAngle(closedLoopTargetAngle);
  }

  public boolean isAtTargetAngle(Rotation2d target, Rotation2d tolerance) {
    var error = inputs.armPosition.minus(target).getRadians();
    return Math.abs(error) <= tolerance.getRadians();
  }

  public boolean isAtTargetAngle(Rotation2d target) {
    return isAtTargetAngle(target, closedLoopTolerance);
  }

  public double getVoltage() {
    return inputs.armAppliedVolts;
  }

  public double getVelocity() {
    return inputs.armVelocity;
  }

  public void setNeutralMode(NeutralModeValue value) {
    io.setNeutralMode(value);
  }
}
