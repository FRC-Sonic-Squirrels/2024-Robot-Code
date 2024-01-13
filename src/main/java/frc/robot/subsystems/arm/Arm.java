// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class Arm extends Command {
  private final ArmIO io;
  private final ArmIOInputs inputs = new ArmIOInputs();

  /** Creates a new ArmSubsystem. */
  public Arm(ArmIO io) {
    this.io = io;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    io.updateInputs(inputs);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
