// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public double getCurrentDraw() {
    return inputs.currentAmps;
  }

  public double getRPM() {
    return inputs.RPM;
  }

  public void setRPM(double RPM) {
    io.setRPM(RPM);
  }

  public void setPercentOut(double percent) {
    io.setPercentOut(percent);
  }

  public Boolean getBeamBreak() {
    return inputs.beamBreak;
  }
}
