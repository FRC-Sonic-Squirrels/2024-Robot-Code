// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class RotateToAngle extends Command {
  private static final String ROOT_TABLE = "RotateToAngle";

  private static final TunableNumberGroup groupTunable = new TunableNumberGroup(ROOT_TABLE);
  private static final LoggedTunableNumber tunableKP = groupTunable.build("kP", 4.9);

  private final DrivetrainWrapper wrapper;
  private final Rotation2d angle;
  private final PIDController rotationalPID = new PIDController(0, 0, 0);

  /** Creates a new RotateToAngle. */
  public RotateToAngle(DrivetrainWrapper wrapper, Rotation2d angle) {
    this.wrapper = wrapper;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationalPID.setP(tunableKP.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrapper.setRotationOverride(
        rotationalPID.calculate(angle.getRadians(), wrapper.getRotationGyroOnly().getRadians()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrapper.resetRotationOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
