// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class HomeMechanism extends Command {
  private Elevator elevator;
  private Arm arm;
  private boolean armReset = false;
  private boolean elevatorReset = false;
  private boolean beginHomingArm = false;
  private final double safeElevatorHeightForArmResetting = 5.0;
  private final double homingVoltage = -0.1;
  private final double homingVelocityMaxToResetElevator = 0.02;
  private final double homingVelocityMaxToResetArm = 0.02;

  /** Creates a new HomeMechanism. */
  public HomeMechanism(Elevator elevator, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, arm);
    setName("HomeMechanism");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armReset) {
      if (!elevatorReset) {
        elevator.setVoltage(homingVoltage);
        if (Math.abs(elevator.getVoltage()) >= Math.abs(homingVoltage) / 2.0
            && Math.abs(elevator.getVelocity()) <= homingVelocityMaxToResetElevator) {
          elevator.resetSensorToHomePosition();
          elevatorReset = true;
        }
      }
    } else {
      if (beginHomingArm) {
        arm.setVoltage(homingVoltage);
        if (Math.abs(arm.getVoltage()) >= Math.abs(homingVoltage) / 2.0
            && Math.abs(arm.getVelocity()) <= homingVelocityMaxToResetArm) {
          arm.resetSensorToHomePosition();
          armReset = true;
        }
      } else {
        elevator.setHeight(safeElevatorHeightForArmResetting);
        if (elevator.isAtTarget(safeElevatorHeightForArmResetting)) {
          beginHomingArm = true;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armReset && elevatorReset;
  }
}
