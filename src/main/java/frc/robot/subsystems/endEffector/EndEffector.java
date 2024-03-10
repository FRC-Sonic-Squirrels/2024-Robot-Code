// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  private static final TunableNumberGroup group = new TunableNumberGroup("EndEffector");

  public static final LoggedTunableNumber distanceToTriggerNoteDetection =
      group.build("distanceToTriggerNote", 11.0);

  private static final LoggedTunableNumber kS = group.build("kS");
  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kV = group.build("kV");
  private static final LoggedTunableNumber ClosedLoopMaxAccelerationConstraint =
      group.build("ClosedLoopMaxAccelerationConstraint");

  /** Creates a new EndEffectorSubsystem. */
  public EndEffector(EndEffectorIO io) {
    this.io = io;

    io.setClosedLoopConstants(
        kP.get(), kV.get(), kS.get(), ClosedLoopMaxAccelerationConstraint.get());
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming("EndEffector")) {
      io.updateInputs(inputs);
      Logger.processInputs("EndEffector", inputs);
      var hc = hashCode();
      if (kS.hasChanged(hc)
          || kP.hasChanged(hc)
          || kV.hasChanged(hc)
          || ClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        io.setClosedLoopConstants(
            kP.get(), kV.get(), kS.get(), ClosedLoopMaxAccelerationConstraint.get());
      }
    }
  }

  public void markStartOfNoteIntaking() {
    io.markStartOfNoteIntaking();
  }

  public void markStartOfNoteDropping() {
    io.markStartOfNoteDropping();
  }

  public void setPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE);
  }

  public double getRPM() {
    return inputs.velocityRPM;
  }

  public boolean intakeSideTOFDetectGamepiece() {
    return intakeSideTOFDistanceInches() <= distanceToTriggerNoteDetection.get();
  }

  public boolean shooterSideTOFDetectGamepiece() {
    return shooterSideTOFDistanceInches() <= distanceToTriggerNoteDetection.get();
  }

  public double intakeSideTOFDistanceInches() {
    return inputs.intakeSideTOFDistanceInches;
  }

  public double shooterSideTOFDistanceInches() {
    return inputs.shooterSideTOFDistanceInches;
  }

  public double noteOffsetInches() {
    return shooterSideTOFDistanceInches() - intakeSideTOFDistanceInches();
  }

  public boolean noteInEndEffector() {
    return intakeSideTOFDetectGamepiece() && shooterSideTOFDetectGamepiece();
  }

  public Command stopCmd() {
    return Commands.runOnce(() -> setPercentOut(0.0), this);
  }

  public void setVelocity(double revPerMin) {
    io.setVelocity(revPerMin);
  }
}
