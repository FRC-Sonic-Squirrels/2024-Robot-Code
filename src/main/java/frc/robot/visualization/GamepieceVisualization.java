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
  private static ArrayList<Pair<Pose3d, Double>> poses = new ArrayList<>(200);
  private static Pose3d[] loggedPoses = new Pose3d[] {};
  private static double gamepieceSpacing = 0.7;
  private static double timeout = 12.0;
  private static Pose2d robotPoseOfShot = new Pose2d();
  private static Translation2d robotVelOfShot = new Translation2d();
  private static Rotation2d shooterAngleOfShot = new Rotation2d();
  private static double shooterRPMofShot = 0.0;
  private static boolean shootingPrev = false;
  private static boolean gamepieceShot = false;
  private static boolean showingPath = false;
  private static boolean prevShowingPath = false;
  private static boolean showPath = false;
  private static boolean initial = true;
  private static GamepieceVisualization instance = new GamepieceVisualization();
  private static Timer shootingTimer = new Timer();
  private boolean shootingGamepiece = false;

  public static GamepieceVisualization getInstance() {
    return instance;
  }

  public void updateVisualization(
      Pose2d robotPose,
      Translation2d robotVel,
      Rotation2d shooterAngle,
      double shooterRPM,
      Boolean shootingGamepiece) {
    this.shootingGamepiece = shootingGamepiece;
    gamepieceShot = shootingGamepiece && !shootingPrev;
    if (gamepieceShot) {
      robotPoseOfShot = robotPose;
      robotVelOfShot = robotVel;
      shooterAngleOfShot = shooterAngle;
      shooterRPMofShot = shooterRPM;
    }
    shootingPrev = shootingGamepiece;
  }

  public void logTraj() {
    Logger.recordOutput("Visualization/shootingGamepiece", shootingGamepiece);
    if (gamepieceShot) shootingTimer.start();
    if (!shootingGamepiece) {
      shootingTimer.stop();
      shootingTimer.reset();
    }
    Translation3d robotToSpeaker =
        Constants.FieldConstants.getSpeakerTranslation3D()
            .minus(GeometryUtil.translation2dTo3d(robotPoseOfShot.getTranslation()));
    double dist =
        Math.sqrt(
            robotToSpeaker.getX() * robotToSpeaker.getX()
                + robotToSpeaker.getY() * robotToSpeaker.getY()
                + robotToSpeaker.getZ() * robotToSpeaker.getZ());
    double shooterTangentialVel =
        shooterRPMofShot
            / 60.0
            * Constants.ShooterConstants.Launcher.WHEEL_DIAMETER_METERS
            * Math.PI;

    Logger.recordOutput("Visualization/ShooterTanSpeed", shooterTangentialVel);

    double horizShooterTangentialVel =
        shooterTangentialVel * Math.cos(shooterAngleOfShot.getRadians());

    Translation3d gamepieceVel =
        new Translation3d(
            -robotVelOfShot.getX()
                + horizShooterTangentialVel * Math.cos(robotPoseOfShot.getRotation().getRadians()),
            -robotVelOfShot.getY()
                + horizShooterTangentialVel * Math.sin(robotPoseOfShot.getRotation().getRadians()),
            shooterTangentialVel * Math.sin(shooterAngleOfShot.getRadians()));
    double gamepieceLinearVel = gamepieceVel.getDistance(new Translation3d());
    Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(gamepieceVel.getY(), gamepieceVel.getX()));
    Rotation2d pitch =
        Rotation2d.fromRadians(
            Math.atan2(gamepieceVel.getZ(), Math.hypot(gamepieceVel.getX(), gamepieceVel.getY())));

    showingPath = !(shootingTimer.get() * gamepieceLinearVel <= dist);
    showPath = showingPath && !prevShowingPath;
    if (!poses.isEmpty())
      poses.set(
          0, new Pair<Pose3d, Double>(new Pose3d(0.0, 0.0, -1000.0, new Rotation3d()), 10000000.0));
    if (shootingGamepiece) {
      if (shootingTimer.get() * gamepieceLinearVel <= dist) {
        if (initial) {
          poses.add(
              0,
              new Pair<Pose3d, Double>(
                  new Pose3d(
                      new Translation3d(
                              shootingTimer.get() * gamepieceLinearVel,
                              new Rotation3d(0.0, -pitch.getRadians(), yaw.getRadians() + Math.PI))
                          .plus(GeometryUtil.translation2dTo3d(robotPoseOfShot.getTranslation())),
                      new Rotation3d(
                          0.0,
                          shooterAngleOfShot.getRadians(),
                          robotPoseOfShot.getRotation().getRadians())),
                  Timer.getFPGATimestamp()));
          initial = false;
        }
        poses.set(
            0,
            new Pair<Pose3d, Double>(
                new Pose3d(
                    new Translation3d(
                            shootingTimer.get() * gamepieceLinearVel,
                            new Rotation3d(0.0, -pitch.getRadians(), yaw.getRadians() + Math.PI))
                        .plus(GeometryUtil.translation2dTo3d(robotPoseOfShot.getTranslation())),
                    new Rotation3d(
                        0.0,
                        shooterAngleOfShot.getRadians(),
                        robotPoseOfShot.getRotation().getRadians())),
                Timer.getFPGATimestamp()));
      } else {
        if (showPath)
          for (int i = 0; i < (int) (dist / gamepieceSpacing) + 3; i++) {
            poses.add(
                new Pair<Pose3d, Double>(
                    new Pose3d(
                        new Translation3d(
                                gamepieceSpacing * i,
                                new Rotation3d(
                                    0.0, -pitch.getRadians(), yaw.getRadians() + Math.PI))
                            .plus(GeometryUtil.translation2dTo3d(robotPoseOfShot.getTranslation())),
                        new Rotation3d(
                            0.0,
                            shooterAngleOfShot.getRadians(),
                            robotPoseOfShot.getRotation().getRadians())),
                    Timer.getFPGATimestamp()));
          }
      }
    }
    for (int i = 0; i < poses.size(); i++) {
      if (Timer.getFPGATimestamp() - poses.get(i).getSecond() >= timeout) poses.remove(i);
    }

    loggedPoses = new Pose3d[poses.size()];
    for (int i = 0; i < poses.size(); i++) {
      loggedPoses[i] = poses.get(i).getFirst();
    }
    prevShowingPath = showingPath;
    Logger.recordOutput("Visualization/GamepieceTraj", loggedPoses);
  }
}
