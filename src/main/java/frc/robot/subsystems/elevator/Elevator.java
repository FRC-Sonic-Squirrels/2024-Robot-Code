// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Elevator/toleranceInches", 0.5);

  private double targetHeightInches = 0.0;

  /** Creates a new ElevatorSubsystem. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setVel(double inchesPerSecond) {
    io.setVel(inchesPerSecond);
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

  public void setPIDConstraints(double kP, double kD, double kG, Constraints constraints) {
    io.setPIDConstraints(kP, kD, kG, constraints);
  }
}
