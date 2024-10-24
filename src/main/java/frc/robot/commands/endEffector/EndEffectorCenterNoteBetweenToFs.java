// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class EndEffectorCenterNoteBetweenToFs extends Command {
  private static final String ROOT_TABLE = "EndEffectorCentering";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal log_difference = logGroup.buildDecimal("difference");
  private static final LoggerEntry.Decimal log_percent = logGroup.buildDecimal("percent");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);
  private static final LoggedTunableNumber kP = group.build("kP", 0.03);
  private static final LoggedTunableNumber kI = group.build("kI", 0.0);
  private static final LoggedTunableNumber kD = group.build("kD", 0.0);
  private static final LoggedTunableNumber tolerance = group.build("toleranceInches", 0.2);

  /** Creates a new EndEffectorCenterNoteBetweenToFs. */
  private final EndEffector endEffector;

  private final Intake intake;
  private final Shooter shooter;

  private final PIDController controller;

  public EndEffectorCenterNoteBetweenToFs(EndEffector endEffector, Intake intake, Shooter shooter) {
    this.endEffector = endEffector;
    this.intake = intake;
    this.shooter = shooter;

    controller = new PIDController(0, 0, 0);

    addRequirements(endEffector);
    setName("EndEffectorCenterNoteBetweenToFs");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    controller.setP(kP.get());
    controller.setI(kI.get());
    controller.setD(kD.get());
    controller.setTolerance(tolerance.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double difference = endEffector.noteOffsetInches();
    log_difference.info(difference);
    if (Double.isFinite(difference)) {
      double percent = controller.calculate(difference, 0.0);
      percent = -MathUtil.clamp(percent, -0.35, 0.35);
      log_percent.info(percent);

      if (Math.abs(percent) <= 0.05) {
        endEffector.setPercentOut(0.0);
        shooter.setPercentOut(0.0);
        intake.setPercentOut(0.0);
      } else {
        endEffector.setVelocity(percent * 2500);
        shooter.setKickerVelocity(percent * 2500);
        intake.setVelocity(percent * 2500);
      }
    } else {
      endEffector.setVelocity(2500);
      intake.setVelocity(2500);
      shooter.setKickerVelocity(-2500);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0.0);
    shooter.setKickerPercentOut(0.0);
    intake.setPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
