// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class EndEffectorCenterNoteBetweenToFs extends Command {
  /** Creates a new EndEffectorCenterNoteBetweenToFs. */
  EndEffector endEffector;

  Intake intake;
  Shooter shooter;

  PIDController controller;

  LoggedTunableNumber kP = new LoggedTunableNumber("EndEffectorCentering/kP", 0.03);
  LoggedTunableNumber kI = new LoggedTunableNumber("EndEffectorCentering/kI", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("EndEffectorCentering/kD", 0.0);
  LoggedTunableNumber tolerance =
      new LoggedTunableNumber("EndEffectorCentering/toleranceInches", 0.2);

  public EndEffectorCenterNoteBetweenToFs(EndEffector endEffector, Intake intake, Shooter shooter) {
    this.endEffector = endEffector;
    this.intake = intake;
    this.shooter = shooter;

    controller = new PIDController(0, 0, 0);

    addRequirements(endEffector);
    setName("EndEffectorCenterNoteBetweenToFs");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    controller.setP(kP.get());
    controller.setI(kI.get());
    controller.setD(kD.get());
    controller.setTolerance(tolerance.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Old Centering Code
    if (Constants.unusedCode) {
      var intakeSideTofSeenGamepiece = endEffector.intakeSideTOFDetectGamepiece();
      var shooterSideTofSeenGamepiece = endEffector.shooterSideTOFDetectGamepiece();

      // maybe note backed up into intake?
      if (!intakeSideTofSeenGamepiece && !shooterSideTofSeenGamepiece) {
        endEffector.setPercentOut(0.3);

      } else if (intakeSideTofSeenGamepiece && !shooterSideTofSeenGamepiece) {
        endEffector.setPercentOut(0.3);

      } else if (!intakeSideTofSeenGamepiece && shooterSideTofSeenGamepiece) {
        endEffector.setPercentOut(-0.3);

      } else if (intakeSideTofSeenGamepiece && shooterSideTofSeenGamepiece) {
        endEffector.setPercentOut(0.0);
      }
    }
    // New Centering Code
    double difference = endEffector.noteOffsetInches();
    Logger.recordOutput("EndEffectorCenter/difference", difference);
    if (Double.isFinite(difference)) {
      double percent = controller.calculate(difference, 0.0);
      percent = -MathUtil.clamp(percent, -0.35, 0.35);
      Logger.recordOutput("EndEffectorCenter/percent", percent);
      endEffector.setPercentOut(percent);
      shooter.setKickerPercentOut(percent);
      intake.setPercentOut(percent);
    } else {
      endEffector.setPercentOut(0.3);
      intake.setPercentOut(0.3);
      shooter.setKickerPercentOut(-0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0.0);
    shooter.setKickerPercentOut(0.0);
    intake.setPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
