// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  /** Creates a new EndEffectorSubsystem. */
  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);
  }

  public double getRPM() {
    return inputs.RPM;
  }

  public void setRPM(double target) {
    io.setRPM(target);
  }

  public void setPercentOut(double percent) {
    io.setVoltage(percent / Constants.MAX_VOLTAGE);
  }

  public Boolean getBeamBreak() {
    return inputs.beamBreak;
  }
}
