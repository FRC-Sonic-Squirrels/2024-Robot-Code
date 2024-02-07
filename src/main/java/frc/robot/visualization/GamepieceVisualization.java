package frc.robot.visualization;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team2930.GeometryUtil;
import frc.robot.Constants;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class GamepieceVisualization {
  private static ArrayList<Pair<Pose3d, Double>> poses = new ArrayList<>();
  private static Pose3d[] loggedPoses = new Pose3d[] {};
  private static double gamepieceSpacing = 0.4572;
  private static boolean prevGamepieceShot = false;
  private static double timeout = 10.0;

  public static void updateVisualization(
      Pose2d robotPose,
      Translation2d robotVel,
      Rotation2d shooterAngle,
      double shooterRPM,
      Boolean gamepieceShot) {
    Translation3d robotToSpeaker =
        Constants.FieldConstants.getSpeakerTranslation3D()
            .minus(GeometryUtil.translation2dTo3d(robotPose.getTranslation()));
    double dist =
        Math.sqrt(
            robotToSpeaker.getX() * robotToSpeaker.getX()
                + robotToSpeaker.getY() * robotToSpeaker.getY()
                + robotToSpeaker.getZ() * robotToSpeaker.getZ());
    double shooterTangentialVel = Constants.ShooterConstants.SHOOTING_SPEED;

    Logger.recordOutput("Visualization/ShooterTanSpeed", shooterTangentialVel);
    double horizShooterTangentialVel = shooterTangentialVel * Math.cos(shooterAngle.getRadians());
    Translation3d gamepieceVel =
        new Translation3d(
            -robotVel.getX()
                + horizShooterTangentialVel * Math.cos(robotPose.getRotation().getRadians()),
            -robotVel.getY()
                + horizShooterTangentialVel * Math.sin(robotPose.getRotation().getRadians()),
            shooterTangentialVel * Math.sin(shooterAngle.getRadians()));
    Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(gamepieceVel.getY(), gamepieceVel.getX()));
    Rotation2d pitch =
        Rotation2d.fromRadians(
            Math.atan2(gamepieceVel.getZ(), Math.hypot(gamepieceVel.getX(), gamepieceVel.getY())));
    if (gamepieceShot && !prevGamepieceShot)
      for (int i = 0; i < (int) (dist / gamepieceSpacing); i++) {
        poses.add(
            new Pair<Pose3d, Double>(
                new Pose3d(
                    new Translation3d(
                            gamepieceSpacing * i,
                            new Rotation3d(0.0, -pitch.getRadians(), yaw.getRadians() + Math.PI))
                        .plus(GeometryUtil.translation2dTo3d(robotPose.getTranslation())),
                    new Rotation3d(
                        0.0, shooterAngle.getRadians(), robotPose.getRotation().getRadians())),
                Timer.getFPGATimestamp()));
      }

    for (int i = 0; i < poses.size(); i++) {
      if (Timer.getFPGATimestamp() - poses.get(i).getSecond() >= timeout) poses.remove(i);
    }

    loggedPoses = new Pose3d[poses.size()];
    for (int i = 0; i < poses.size(); i++) {
      loggedPoses[i] = poses.get(i).getFirst();
    }
    prevGamepieceShot = gamepieceShot;
  }

  public static void logTraj() {
    Logger.recordOutput("Visualization/GamepieceTraj", loggedPoses);
  }
}
