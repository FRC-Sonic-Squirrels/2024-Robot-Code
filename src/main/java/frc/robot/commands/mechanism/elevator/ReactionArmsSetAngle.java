// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanism.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.elevator.Elevator;

public class ReactionArmsSetAngle extends Command {
  /** Creates a new ReactionArmsSetAngle. */
  Elevator elevator;

  double rotations;
  Double nonDefaultTolerance = Double.NaN;

  TunableNumberGroup group = new TunableNumberGroup("ReactionArms");
  LoggedTunableNumber tolerance = group.build("tolerance", 0.5);

  public ReactionArmsSetAngle(Elevator elevator, double rotations) {
    this.elevator = elevator;
    this.rotations = rotations;

    addRequirements(elevator);
    setName("ReactionArmsSetAngle");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setReactionArmsAngle(rotations);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(rotations - elevator.getReationsArmsRotations()) <= tolerance.get();
  }
}
