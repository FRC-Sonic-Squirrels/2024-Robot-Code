// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ControlMode;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private static final String ROOT_TABLE = "Arm";
  private static final LoggedTunableNumber kP = new LoggedTunableNumber(ROOT_TABLE + "/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber(ROOT_TABLE + "/kD");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber(ROOT_TABLE + "/kG");

  private static final LoggedTunableNumber closedLoopMaxVelocityConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/defaultClosedLoopMaxVelocityConstraint");
  private static final LoggedTunableNumber closedLoopMaxAccelerationConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/defaultClosedLoopMaxAccelerationConstraint");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024) {
      kP.initDefault(56.0);
      kD.initDefault(0);
      kG.initDefault(0.28);

      // FIXME: find the theoritical from the JVN docs
      closedLoopMaxVelocityConstraint.initDefault(80);
      closedLoopMaxAccelerationConstraint.initDefault(160);
    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {

      kP.initDefault(2.5);
      kD.initDefault(0);
      kG.initDefault(1.485);

      closedLoopMaxVelocityConstraint.initDefault(40);
      closedLoopMaxAccelerationConstraint.initDefault(80);
    }
  }

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private ControlMode currentControlMode = ControlMode.OPEN_LOOP;
  private Rotation2d closedLoopTargetAngle = new Rotation2d();
  // FIXME: tune this value
  private final Rotation2d closedLoopTolerance = Rotation2d.fromDegrees(1);
  private final double MAX_VOLTAGE = 12;

  /** Creates a new ArmSubsystem. */
  public Arm(ArmIO io) {
    this.io = io;

    io.resetSensorPosition(Constants.ArmConstants.HOME_POSITION);
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
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    Logger.recordOutput("Arm/PositionDegrees", inputs.armPosition.getDegrees() % 360);
    Logger.recordOutput("Arm/ControlMode", currentControlMode);
    Logger.recordOutput("Arm/targetClosedLoopSetpointDegrees", closedLoopTargetAngle.getDegrees());

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
    return isAtTargetAngle(closedLoopTolerance);
  }

  public boolean isAtTargetAngle(Rotation2d tolerance) {
    if (currentControlMode == ControlMode.OPEN_LOOP) {
      return false;
    }
    var error = inputs.armPosition.minus(closedLoopTargetAngle).getRadians();
    return Math.abs(error) <= tolerance.getRadians();
  }
}
