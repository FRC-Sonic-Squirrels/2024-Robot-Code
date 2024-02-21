// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanism;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class HomeMechanism extends Command {
  private Elevator elevator;
  private Arm arm;
  private boolean armReset = false;
  private boolean elevatorReset = false;
  private boolean beginHomingArm = false;

  private static final TunableNumberGroup group = new TunableNumberGroup("HomeMechanism");

  private static final LoggedTunableNumber safeElevatorHeightForArmResetting =
      group.build("safeElevatorHeightForArmResetting", 5.0);

  private static final LoggedTunableNumber homingVoltageElevator =
      group.build("homingVoltageElevator", -0.1);

  private static final LoggedTunableNumber homingVoltageArm = group.build("homingVoltageArm", -0.1);

  private static final LoggedTunableNumber homingVelocityMaxToResetElevator =
      group.build("homingVelocityMaxToResetElevator", 0.02);

  private static final LoggedTunableNumber homingVelocityMaxToResetArm =
      group.build("homingVelocityMaxToResetArm", 0.02);

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
        elevator.setVoltage(homingVoltageElevator.get());
        if (Math.abs(elevator.getVoltage()) >= Math.abs(homingVoltageElevator.get()) / 2.0
            && Math.abs(elevator.getVelocity()) <= homingVelocityMaxToResetElevator.get()) {
          elevator.resetSensorToHomePosition();
          elevatorReset = true;
        }
      }
    } else {
      if (beginHomingArm) {
        arm.setVoltage(homingVoltageArm.get());
        if (Math.abs(arm.getVoltage()) >= Math.abs(homingVoltageArm.get()) / 2.0
            && Math.abs(arm.getVelocity()) <= homingVelocityMaxToResetArm.get()) {
          arm.resetSensorToHomePosition();
          armReset = true;
        }
      } else {
        Measure<Distance> height = Units.Inches.of(safeElevatorHeightForArmResetting.get());
        elevator.setHeight(height);
        if (elevator.isAtTarget(height)) {
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
