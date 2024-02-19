// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EndEffector;

public class EndEffectorCenterNoteBetweenToFs extends Command {
  /** Creates a new EndEffectorCenterNoteBetweenToFs. */
  EndEffector endEffector;

  public EndEffectorCenterNoteBetweenToFs(EndEffector endEffector) {
    this.endEffector = endEffector;

    addRequirements(endEffector);
    setName("EndEffectorCenterNoteBetweenToFs");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endEffector.intakeSideTOFDetectGamepiece()
        && endEffector.shooterSideTOFDetectGamepiece();
  }
}
