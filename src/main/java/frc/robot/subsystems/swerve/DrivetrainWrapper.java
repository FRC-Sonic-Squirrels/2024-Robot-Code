package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class DrivetrainWrapper {
  private final Drivetrain drivetrain;
  private ChassisSpeeds chassisSpeedsBase = new ChassisSpeeds();
  private ChassisSpeeds chassisSpeedsOverride;
  private double omegaOverride = Double.NaN;

  public DrivetrainWrapper(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  /**
   * Sets robot-centric chassis speeds
   *
   * @param chassisSpeeds speeds (m/s, rad/s)
   */
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

    drivetrain.runVelocity(chassisSpeeds, prioritizeRotation);
  }

  public Subsystem getRequirements() {
    return drivetrain;
  }

  public Pose2d getPoseEstimatorPoseWithGyroOnlyRotation() {
    return new Pose2d(drivetrain.getPoseEstimatorPose().getTranslation(), getRotationGyroOnly());
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return drivetrain.getMaxLinearSpeedMetersPerSec();
  }

  public double getMaxAngularSpeedRadPerSec() {
    return drivetrain.getMaxAngularSpeedRadPerSec();
  }

  public Pose2d getFieldRelativeVelocities() {
    return drivetrain.getFieldRelativeVelocities();
  }

  public Rotation2d getRotationGyroOnly() {
    return drivetrain.getRotationGyroOnly();
  }

  public SysIdRoutine.Mechanism getSysIdMechanism() {
    return new SysIdRoutine.Mechanism(
        (voltage) -> drivetrain.runCharacterizationVolts(voltage.in(Units.Volts)),
        null, // No log consumer, since data is recorded by AdvantageKit
        drivetrain);
  }
}
