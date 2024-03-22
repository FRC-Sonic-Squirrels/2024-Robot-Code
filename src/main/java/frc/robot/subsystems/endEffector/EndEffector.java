// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;

public class EndEffector extends SubsystemBase {
  private static final String ROOT_TABLE = "EndEffector";

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  private static final LoggerGroup logInputs = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_velocityRPM =
      logInputs.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal logInputs_appliedVolts =
      logInputs.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_currentAmps =
      logInputs.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_tempCelsius =
      logInputs.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_intakeSideTOFDistanceInches =
      logInputs.buildDecimal("IntakeSideTOFDistanceInches");
  private static final LoggerEntry.Decimal logInputs_shooterSideTOFDistanceInches =
      logInputs.buildDecimal("ShooterSideTOFDistanceInches");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  public static final LoggedTunableNumber distanceToTriggerNoteDetection =
      group.build("distanceToTriggerNote", 11.0);

  private static final LoggedTunableNumber kS = group.build("kS");
  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kV = group.build("kV");
  private static final LoggedTunableNumber ClosedLoopMaxAccelerationConstraint =
      group.build("ClosedLoopMaxAccelerationConstraint");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_MAESTRO) {
      kP.initDefault(0.4);
      kV.initDefault(0.15);
      ClosedLoopMaxAccelerationConstraint.initDefault(300.0);
    } else if (Constants.RobotMode.isSimBot()) {

      kP.initDefault(0.0);
      kV.initDefault(0.0);
      ClosedLoopMaxAccelerationConstraint.initDefault(0.0);
    }
  }

  private final EndEffectorIO io;
  private final EndEffectorIO.Inputs inputs = new EndEffectorIO.Inputs();

  /** Creates a new EndEffectorSubsystem. */
  public EndEffector(EndEffectorIO io) {
    this.io = io;

    updateConstants(io);
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      io.updateInputs(inputs);
      logInputs_velocityRPM.info(inputs.velocityRPM);
      logInputs_appliedVolts.info(inputs.appliedVolts);
      logInputs_currentAmps.info(inputs.currentAmps);
      logInputs_tempCelsius.info(inputs.tempCelsius);
      logInputs_intakeSideTOFDistanceInches.info(inputs.intakeSideTOFDistanceInches);
      logInputs_shooterSideTOFDistanceInches.info(inputs.shooterSideTOFDistanceInches);

      var hc = hashCode();
      if (kS.hasChanged(hc)
          || kP.hasChanged(hc)
          || kV.hasChanged(hc)
          || ClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        updateConstants(io);
      }
    }
  }

  private void updateConstants(EndEffectorIO io) {
    io.setClosedLoopConstants(
        kP.get(), kV.get(), kS.get(), ClosedLoopMaxAccelerationConstraint.get());
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
