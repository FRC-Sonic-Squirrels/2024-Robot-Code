// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ControlMode;
import frc.robot.Constants.RobotMode.RobotType;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private static final String ROOT_TABLE = "Wrist";
  private static final LoggedTunableNumber kP = new LoggedTunableNumber(ROOT_TABLE + "/kP", 10.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber(ROOT_TABLE + "/kD", 0.0);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber(ROOT_TABLE + "/kG", 0.0);

  private static final LoggedTunableNumber closedLoopMaxVelocityConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/defaultClosedLoopMaxVelocityConstraint", 10.0);
  private static final LoggedTunableNumber closedLoopMaxAccelerationConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/defaultClosedLoopMaxAccelerationConstraint", 10.0);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024) {
      kP.initDefault(0.0);
      kD.initDefault(0);
      kG.initDefault(0);

      closedLoopMaxVelocityConstraint.initDefault(0);
      closedLoopMaxAccelerationConstraint.initDefault(0);
    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {

      kP.initDefault(2.5);
      kD.initDefault(0);
      kG.initDefault(1.485);

      closedLoopMaxVelocityConstraint.initDefault(40);
      closedLoopMaxAccelerationConstraint.initDefault(80);
    }
  }

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private ControlMode currentControlMode = ControlMode.POSITION;
  private Rotation2d closedLoopTargetAngle = new Rotation2d();
  private final Rotation2d closedLoopTolerance = Rotation2d.fromDegrees(2.0);
  private final double MAX_VOLTAGE = 12.0;

  /** Creates a new WristSubsystem. */
  public Wrist(WristIO io) {
    this.io = io;

    io.resetSensorPosition(Constants.WristConstants.HOME_POSITION);
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
    Logger.processInputs("Wrist", inputs);

    // ---- UPDATE TUNABLE NUMBERS
    var hc = hashCode();
    if (kP.hasChanged(hc)
        || kD.hasChanged(hc)
        || kG.hasChanged(hc)
        || closedLoopMaxVelocityConstraint.hasChanged(hc)
        || closedLoopMaxAccelerationConstraint.hasChanged(hc)) {
      io.setClosedLoopConstants(
          kP.get(),
          kP.get(),
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
                Constants.WristConstants.MIN_WRIST_ANGLE.getRadians(),
                Constants.WristConstants.MAX_WRIST_ANGLE.getRadians()));

    currentControlMode = ControlMode.POSITION;
    closedLoopTargetAngle = angle;
    io.setClosedLoopPosition(angle);
  }

  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -MAX_VOLTAGE, MAX_VOLTAGE);
    currentControlMode = ControlMode.VOLTAGE;
    io.setVoltage(volts);
  }

  public Rotation2d getAngle() {
    return inputs.angle;
  }

  public void resetSensorToHomePosition() {
    io.resetSensorPosition(Constants.WristConstants.MIN_WRIST_ANGLE);
  }

  public boolean isAtTargetAngle() {
    return isAtTargetAngle(closedLoopTolerance);
  }

  public boolean isAtTargetAngle(Rotation2d tolerance) {
    if (currentControlMode == ControlMode.VOLTAGE) {
      return false;
    }

    double error = inputs.angle.minus(closedLoopTargetAngle).getRadians();
    return error <= tolerance.getRadians();
  }
}
