// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static final String ROOT_TABLE = "Elevator";
  private static final LoggedTunableNumber kP = new LoggedTunableNumber(ROOT_TABLE + "/kP", 0.5);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber(ROOT_TABLE + "/kD", 0.0);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber(ROOT_TABLE + "/kG", 1.0);

  private static final LoggedTunableNumber closedLoopMaxVelocityConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/defaultClosedLoopMaxVelocityConstraint", 10.0);
  private static final LoggedTunableNumber closedLoopMaxAccelerationConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/defaultClosedLoopMaxAccelerationConstraint", 10.0);

  private final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Elevator/toleranceInches", 0.5);

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
            kP.get(),
            kG.get(),
            new Constraints(
                closedLoopMaxVelocityConstraint.get(), closedLoopMaxAccelerationConstraint.get()));
      }
    }
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE);
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

  public double getInchesFromRotations(double rotations) {
    double inches = 0.0;
    inches =
        Units.metersToInches(rotations / ElevatorConstants.GEAR_RATIO)
            * 2
            * Math.PI
            * ElevatorConstants.WHEEL_RADIUS;
    return inches;
  }
}
