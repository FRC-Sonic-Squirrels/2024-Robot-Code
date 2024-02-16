// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private static final String ROOT_TABLE = "Elevator";
  private static final LoggedTunableNumber kP = new LoggedTunableNumber(ROOT_TABLE + "/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber(ROOT_TABLE + "/kD");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber(ROOT_TABLE + "/kG");

  private static final LoggedTunableNumber closedLoopMaxVelocityConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/defaultClosedLoopMaxVelocityConstraint");
  private static final LoggedTunableNumber closedLoopMaxAccelerationConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/defaultClosedLoopMaxAccelerationConstraint");

  private final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Elevator/toleranceInches", 0.1);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {
      kP.initDefault(0.0);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      closedLoopMaxVelocityConstraint.initDefault(0.0);
      closedLoopMaxAccelerationConstraint.initDefault(0.0);

    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024) {
      kP.initDefault(0.0);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      closedLoopMaxVelocityConstraint.initDefault(0.0);
      closedLoopMaxAccelerationConstraint.initDefault(0.0);
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double targetHeightInches = 0.0;

  /** Creates a new ElevatorSubsystem. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    io.setPIDConstraints(
        kP.get(),
        kD.get(),
        kG.get(),
        new Constraints(
            closedLoopMaxVelocityConstraint.get(), closedLoopMaxAccelerationConstraint.get()));
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming("Elevator")) {
      io.updateInputs(inputs);
      Logger.processInputs("Elevator", inputs);

      // ---- UPDATE TUNABLE NUMBERS
      var hc = hashCode();
      if (kP.hasChanged(hc)
          || kD.hasChanged(hc)
          || kG.hasChanged(hc)
          || closedLoopMaxVelocityConstraint.hasChanged(hc)
          || closedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        io.setPIDConstraints(
            kP.get(),
            kD.get(),
            kG.get(),
            new Constraints(
                closedLoopMaxVelocityConstraint.get(), closedLoopMaxAccelerationConstraint.get()));
      }
    }
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setHeight(double heightInches) {
    io.setHeight(heightInches);
    targetHeightInches = heightInches;
  }

  public boolean isAtTarget() {
    return Math.abs(targetHeightInches - inputs.heightInches) <= tolerance.get();
  }

  public boolean isAtTarget(double heightInches) {
    return Math.abs(heightInches - inputs.heightInches) <= tolerance.get();
  }

  public double getHeightInches() {
    return inputs.heightInches;
  }

  public void resetSensorToHomePosition() {
    io.setSensorPositionInches(0.0);
  }
}
