// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EndEffector;
import java.util.function.DoubleSupplier;

public class EndEffectorPercentOut extends Command {
  EndEffector endEffector;
  DoubleSupplier percentSupplier;
  double percent;

  /** Creates a new EndEffectorPercentOut. */
  public EndEffectorPercentOut(EndEffector endEffector, DoubleSupplier percentSupplier) {
    this.endEffector = endEffector;
    this.percentSupplier = percentSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endEffector);
  }

  public EndEffectorPercentOut(EndEffector endEffector, double percent) {
    this(endEffector, () -> percent);
    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setPercentOut(percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
