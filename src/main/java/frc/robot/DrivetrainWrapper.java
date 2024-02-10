package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class DrivetrainWrapper {
  private final Drivetrain drivetrain;
  private ChassisSpeeds chassisSpeedsBase;
  private ChassisSpeeds chassisSpeedsOverride;
  private double omegaOverride = Double.NaN;

  public DrivetrainWrapper(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public void setVelocity(ChassisSpeeds chassisSpeeds) {
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
    Logger.recordOutput("DrivetrainWrapper/chassisSpeedsBase", chassisSpeedsBase);
    Logger.recordOutput("DrivetrainWrapper/chassisSpeedsOverride", chassisSpeedsOverride);
    Logger.recordOutput("DrivetrainWrapper/omegaOverride", omegaOverride);

    ChassisSpeeds chassisSpeeds;

    if (chassisSpeedsOverride != null) {
      chassisSpeeds = chassisSpeedsOverride;
    } else {
      chassisSpeeds = chassisSpeedsBase;
    }

    if (Double.isFinite(omegaOverride)) {
      chassisSpeeds =
          new ChassisSpeeds(
              chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, omegaOverride);
    }

    // Convert to field relative speeds & send command
    drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            omegaOverride,
            drivetrain.getRotation()));
  }

  public Pose2d getPoseEstimatorPose() {
    return drivetrain.getPoseEstimatorPose();
  }
}
