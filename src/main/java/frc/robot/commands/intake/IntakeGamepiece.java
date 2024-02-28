// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Consumer;

public class IntakeGamepiece extends Command {
  private Intake intake;
  private EndEffector endEffector;
  private Shooter shooter;

  private static final TunableNumberGroup group = new TunableNumberGroup("IntakeGamepiece");

  private LoggedTunableNumber rumbleIntensityPercent = group.build("rumbleIntensityPercent", 0.3);

  private LoggedTunableNumber intakingPercent = group.build("intakingPercent", 0.4);
  private LoggedTunableNumber intakingPercentWithGamepiece =
      group.build("intakingPercentWithGamepiece", 0.3);

  private final Consumer<Double> rumbleConsumer;

  /** Creates a new IntakeDefaultIdleRPM. */
  public IntakeGamepiece(
      Intake intake, EndEffector endEffector, Shooter shooter, Consumer<Double> rumbleConsumer) {
    this.intake = intake;
    this.endEffector = endEffector;
    this.shooter = shooter;
    this.rumbleConsumer = rumbleConsumer;

    addRequirements(intake);
    setName("IntakeGamepiece");
  }

  public IntakeGamepiece(Intake intake, EndEffector endEffector, Shooter shooter) {
    this(intake, endEffector, shooter, (ignore) -> {});
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var intakeSideTofSeenGamepiece = endEffector.intakeSideTOFDetectGamepiece();
    var shooterSideTofSeenGamepiece = endEffector.shooterSideTOFDetectGamepiece();
    var rumbleValue = rumbleIntensityPercent.get();

    if (!shooterSideTofSeenGamepiece) {
      shooter.setKickerPercentOut(0.0);

      if (!intakeSideTofSeenGamepiece) {
        endEffector.markStartOfNoteIntaking();

        double percent = intakingPercent.get();
        intake.setPercentOut(percent);
        endEffector.setPercentOut(percent);

        // dont rumble if we dont see gamepiece
        rumbleValue = 0.0;

        // if we see the game piece slow down system to maintain better control over the note
      } else {
        double percent = intakingPercentWithGamepiece.get();
        intake.setPercentOut(percent);
        endEffector.setPercentOut(percent);
      }
    } else {
      intake.setPercentOut(0.0);
      if (!intakeSideTofSeenGamepiece) {
        endEffector.setPercentOut(EndEffectorConstants.CENTERING_NOTE_REVERSE);
        shooter.setKickerPercentOut(-0.1);
      } else {
        endEffector.setPercentOut(0.0);
        shooter.setKickerPercentOut(0.0);
      }
    }

    rumbleConsumer.accept(rumbleValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPercentOut(0.0);
    endEffector.setPercentOut(0.0);
    shooter.setKickerPercentOut(0.0);

    rumbleConsumer.accept(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endEffector.noteInEndEffector();
  }
}
