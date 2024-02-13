package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class DrivetrainWrapper {
  private final Drivetrain drivetrain;
  private ChassisSpeeds chassisSpeedsBase = new ChassisSpeeds();
  private ChassisSpeeds chassisSpeedsOverride;
  private double omegaOverride = Double.NaN;

  public DrivetrainWrapper(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public void setVelocity(ChassisSpeeds chassisSpeeds) {
    if (chassisSpeeds == null) chassisSpeeds = new ChassisSpeeds();
    chassisSpeedsBase = chassisSpeeds;
  }

  public void setVelocityOverride(ChassisSpeeds chassisSpeeds) {
    chassisSpeedsOverride = chassisSpeeds;
  }

  public void resetVelocityOverride() {
    chassisSpeedsOverride = null;
  }

  public void setRotationOverride(double omega) {
    omegaOverride = omega;
  }

  public void resetRotationOverride() {
    omegaOverride = Double.NaN;
  }

  public void apply() {
    if (chassisSpeedsBase != null) {
      Logger.recordOutput("DrivetrainWrapper/chassisSpeedsBase", chassisSpeedsBase);
    }
    if (chassisSpeedsOverride != null) {
      Logger.recordOutput("DrivetrainWrapper/chassisSpeedsOverride", chassisSpeedsOverride);
    }
    Logger.recordOutput("DrivetrainWrapper/omegaOverride", omegaOverride);

    ChassisSpeeds chassisSpeeds;

    if (chassisSpeedsOverride != null) {
      chassisSpeeds = chassisSpeedsOverride;
    } else {
      chassisSpeeds = chassisSpeedsBase;
    }

    boolean prioritizeRotation;

    if (Double.isFinite(omegaOverride)) {
      prioritizeRotation = true;
      chassisSpeeds =
          new ChassisSpeeds(
              chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, omegaOverride);
    } else {
      prioritizeRotation = false;
    }
    // Convert to field relative speeds & send command
    ChassisSpeeds robotChassisSpeed =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            omegaOverride,
            drivetrain.getRotation());

    drivetrain.runVelocity(robotChassisSpeed, prioritizeRotation);
  }

  public Subsystem getRequirements() {
    return drivetrain;
  }

  public Pose2d getPoseEstimatorPose() {
    return drivetrain.getPoseEstimatorPose();
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return drivetrain.getMaxLinearSpeedMetersPerSec();
  }

  public double getMaxAngularSpeedRadPerSec() {
    return drivetrain.getMaxAngularSpeedRadPerSec();
  }
}
