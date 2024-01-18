// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

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

  /** Creates a new ArmSubsystem. */
  public Arm(ArmIO io) {
    this.io = io;

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
    io.setClosedLoopPosition(angle);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.armPositionRad);
  }
}
