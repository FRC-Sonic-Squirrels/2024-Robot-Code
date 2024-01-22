// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class IntakeDefaultCommand extends Command {
  private Intake intake;
  private XboxController controller;
  private Timer gamepieceTimeInIntake = new Timer();

  private LoggedTunableNumber rumbleDurationSeconds =
      new LoggedTunableNumber("IntakeDefaultCommand/rumbleDurationSeconds", 0.4);
  private LoggedTunableNumber rumbleIntensityPercent =
      new LoggedTunableNumber("IntakeDefaultCommand/rumbleIntensityPercent", 0.3);

  /** Creates a new IntakeDefaultIdleRPM. */
  public IntakeDefaultCommand(Intake intake, XboxController controller) {
    this.intake = intake;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getBeamBreak()) {
      gamepieceTimeInIntake.start();
    } else {
      gamepieceTimeInIntake.stop();
      gamepieceTimeInIntake.reset();
    }
    if (gamepieceTimeInIntake.get() >= 0.01
        && gamepieceTimeInIntake.get() <= rumbleDurationSeconds.get()) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensityPercent.get());
    }
    intake.setPercentOut(Constants.IntakeConstants.INTAKE_IDLE_PERCENT_OUT);
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
