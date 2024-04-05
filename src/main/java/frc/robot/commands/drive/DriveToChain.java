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

public class DriveToChain extends Command {
  private final VisionGamepiece visionGamepiece;
  private final DrivetrainWrapper drive;
  private final Supplier<Boolean> rotationWithinTolerance;

  private static TunableNumberGroup group = new TunableNumberGroup("DriveToChainFast");
  private static LoggedTunableNumber kp = group.build("kp", 0.02);
  private static LoggedTunableNumber offsetTolerance = group.build("xOffsetTolerancePixels", 3);
  private static LoggedTunableNumber driveInSpeed = group.build("driveInSpeed", 1.5);

  private PIDController xOffsetController;

  /** Creates a new DriveToChain. */
  public DriveToChain(
      VisionGamepiece visionGamepiece,
      DrivetrainWrapper drive,
      Supplier<Boolean> rotationWithinTolerance) {
    this.visionGamepiece = visionGamepiece;
    this.drive = drive;
    this.rotationWithinTolerance = rotationWithinTolerance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xOffsetController.setP(kp.get());
    xOffsetController.setI(0.0);
    xOffsetController.setD(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tagOffset = visionGamepiece.getTagOffset();

    drive.setVelocityOverride(
        new ChassisSpeeds(
            rotationWithinTolerance.get() && Math.abs(tagOffset) < offsetTolerance.get()
                ? driveInSpeed.get()
                : 0.0,
            xOffsetController.calculate(tagOffset, 0),
            0.0));
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
}
