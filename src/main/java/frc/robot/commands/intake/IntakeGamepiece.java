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

public class IntakeGamepiece extends Command {
  private Intake intake;
  private XboxController controller;
  private Timer timeSinceLastGamepiece = new Timer();

  private boolean controllerRumbled = false;
  private boolean beamBreakPrev = false;

  private LoggedTunableNumber rumbleDurationSeconds =
      new LoggedTunableNumber("IntakeGamepiece/rumbleDurationSeconds", 0.4);
  private LoggedTunableNumber rumbleIntensityPercent =
      new LoggedTunableNumber("IntakeGamepiece/rumbleIntensityPercent", 0.3);

  /** Creates a new IntakeDefaultIdleRPM. */
  public IntakeGamepiece(Intake intake, XboxController controller) {
    this.intake = intake;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    setName("IntakeGamepiece");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getBeamBreak() && !beamBreakPrev) {
      timeSinceLastGamepiece.reset();
      timeSinceLastGamepiece.start();
    }
    beamBreakPrev = intake.getBeamBreak();
    if (timeSinceLastGamepiece.get() >= 0.01
        && timeSinceLastGamepiece.get() <= rumbleDurationSeconds.get()) {
      if (controller != null)
        controller.setRumble(RumbleType.kBothRumble, rumbleIntensityPercent.get());
      controllerRumbled = true;
    } else if (controllerRumbled) {
      if (controller != null) controller.setRumble(RumbleType.kBothRumble, 0.0);
      controllerRumbled = false;
    }
    intake.setPercentOut(Constants.IntakeConstants.INTAKE_IDLE_PERCENT_OUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
