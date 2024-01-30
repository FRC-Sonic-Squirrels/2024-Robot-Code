// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import java.util.function.Supplier;

public class ArmSetAngle extends Command {
  /** Creates a new ArmSetAngle. */
  Arm arm;

  Supplier<Rotation2d> angleSupplier;

  public ArmSetAngle(Arm arm, Rotation2d angle) {
    this(arm, () -> angle);
  }

  public ArmSetAngle(Arm arm, Supplier<Rotation2d> angleSupplier) {
    this.arm = arm;
    this.angleSupplier = angleSupplier;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setAngle(angleSupplier.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isAtTargetAngle();
  }
}
