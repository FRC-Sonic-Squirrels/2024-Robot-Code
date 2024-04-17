// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import java.util.function.Supplier;

public class AlignWithChain extends Command {
  private final VisionGamepiece visionGamepiece;
  private final DrivetrainWrapper drive;
  private final Supplier<Double> rotationOffset;

  private static TunableNumberGroup group = new TunableNumberGroup("DriveToChainFast");
  private static LoggedTunableNumber kp = group.build("kp", 0.06);
  private static LoggedTunableNumber offsetTolerance = group.build("xOffsetTolerancePixels", 1.5);
  private static LoggedTunableNumber driveInSpeed = group.build("driveInSpeed", 1);

  private PIDController xOffsetController = new PIDController(0, 0, 0);
  private boolean hasSeenStageTag;

  private boolean driving = false;

  private double tagOffset = 0;

  /** Creates a new DriveToChain. */
  public AlignWithChain(
      VisionGamepiece visionGamepiece, DrivetrainWrapper drive, Supplier<Double> rotationOffset) {
    this.visionGamepiece = visionGamepiece;
    this.drive = drive;
    this.rotationOffset = rotationOffset;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xOffsetController.setP(kp.get());
    xOffsetController.setI(0.0);
    xOffsetController.setD(0.0);
    hasSeenStageTag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (visionGamepiece.seesStageTags()) {
      tagOffset = visionGamepiece.getTagYaw();
      hasSeenStageTag = true;
    }

    driving = hasSeenStageTag;

    double effort = xOffsetController.calculate(tagOffset, 0) / Math.max(rotationOffset.get(), 1.0);

    if (Math.abs(tagOffset) < 1) {
      effort *= 2;
    }

    drive.setVelocityOverride(new ChassisSpeeds(0.0, driving ? effort : 0.0, 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.resetVelocityOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean driving() {
    return driving;
  }
}
