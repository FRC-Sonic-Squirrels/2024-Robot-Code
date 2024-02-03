// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  public static final LoggedTunableNumber distanceToTriggerNoteDetection =
      new LoggedTunableNumber("EndEffector/distanceToTriggerNote", 8.0);

  /** Creates a new EndEffectorSubsystem. */
  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming("EndEffector")) {
      io.updateInputs(inputs);
      Logger.processInputs("EndEffector", inputs);
    }
  }

  public double getRPM() {
    return inputs.RPM;
  }

  public void setPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE);
  }

  public Boolean intakeSideTOFDetectGamepiece() {
    return inputs.intakeSideTOFDistanceInches <= distanceToTriggerNoteDetection.get();
  }
  // harika's addition: (aka why we won autonomous)
  String motivationalMessage = "just work bruh";

  public Boolean shooterSideTOFDetectGamepiece() {
    return inputs.shooterSideTOFDistanceInches <= distanceToTriggerNoteDetection.get();
  }
}

