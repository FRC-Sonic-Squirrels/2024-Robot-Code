package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;

public class DrivetrainWrapper {
  private static final LoggerGroup logGroup = LoggerGroup.build("DrivetrainWrapper");
  private static final LoggerEntry.Struct<ChassisSpeeds> logChassisSpeedsBase =
      logGroup.buildStruct(ChassisSpeeds.class, "chassisSpeedsBase");
  private static final LoggerEntry.Struct<ChassisSpeeds> logChassisSpeedsOverride =
      logGroup.buildStruct(ChassisSpeeds.class, "chassisSpeedsOverride");
  private static final LoggerEntry.Decimal logOmegaOverride =
      logGroup.buildDecimal("omegaOverride");
  private static final LoggerEntry.Decimal logGyroDrift = logGroup.buildDecimal("gyroDrift");

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
      logChassisSpeedsBase.info(chassisSpeedsBase);
    }
    if (chassisSpeedsOverride != null) {
      logChassisSpeedsOverride.info(chassisSpeedsOverride);
    }
    logOmegaOverride.info(omegaOverride);

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

    var pose1 = getPoseEstimatorPose(false);
    var pose2 = getPoseEstimatorPose(true);

    logGyroDrift.info(pose1.getRotation().minus(pose2.getRotation()));
  }

  public Subsystem getRequirements() {
    return drivetrain;
  }

  public Pose2d getPoseEstimatorPose(boolean prioritizeGyro) {
    var pose = drivetrain.getPoseEstimatorPose();
    if (prioritizeGyro) {
      pose = new Pose2d(pose.getTranslation(), getRotationGyroOnly());
    }

    return pose;
  }

  public Pose2d getPoseEstimatorPoseStageBlue(boolean prioritizeGyro) {
    var pose = drivetrain.getPoseEstimatorPoseStageBlue();
    if (prioritizeGyro) {
      pose = new Pose2d(pose.getTranslation(), getRotationGyroOnly());
    }

    return pose;
  }

  public Pose2d getPoseEstimatorPoseStageRed(boolean prioritizeGyro) {
    var pose = drivetrain.getPoseEstimatorPoseStageRed();
    if (prioritizeGyro) {
      pose = new Pose2d(pose.getTranslation(), getRotationGyroOnly());
    }

    return pose;
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
