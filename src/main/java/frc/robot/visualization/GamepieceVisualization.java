package frc.robot.visualization;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import java.util.ArrayList;

public class GamepieceVisualization {
  private static final LoggerGroup group = new LoggerGroup("Visualization");
  private static final LoggerEntry logShootingGamepiece = group.build("shootingGamepiece");
  private static final LoggerEntry logShooterTanSpeed = group.build("ShooterTanSpeed");
  private static final LoggerEntry logGamepieceTraj = group.build("GamepieceTraj");

  private static final ArrayList<Pair<Pose3d, Double>> poses = new ArrayList<>(200);
  private static Pose3d[] loggedPoses = new Pose3d[] {};
  private static final double gamepieceSpacing = 0.7;
  private static final double timeout = 12.0;
  private static Pose2d robotPoseOfShot = new Pose2d();
  private static Translation2d robotVelOfShot = Constants.zeroTranslation2d;
  private static Rotation2d shooterAngleOfShot = Constants.zeroRotation2d;
  private static double shooterRPMofShot = 0.0;
  private static final boolean shootingPrev = false;
  private static final boolean gamepieceShot = false;
  private static boolean showingPath = false;
  private static boolean prevShowingPath = false;
  private static boolean showPath = false;
  private static boolean initial = true;
  private static final GamepieceVisualization instance = new GamepieceVisualization();
  private static final Timer shootingTimer = new Timer();
  private final boolean shootingGamepiece = false;

  public static GamepieceVisualization getInstance() {
    return instance;
  }

  public void updateVisualization(
      Pose2d robotPose, Translation2d robotVel, Rotation2d shooterAngle, double shooterRPM) {
    robotPoseOfShot = robotPose;
    robotVelOfShot = robotVel;
    shooterAngleOfShot = shooterAngle;
    shooterRPMofShot = shooterRPM;
    shootingTimer.reset();
    shootingTimer.start();
  }

  public void logTraj() {
    double shootingTime = shootingTimer.get();
    logShootingGamepiece.info(shootingGamepiece);

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
            * Constants.ShooterConstants.Launcher.WHEEL_DIAMETER.in(Units.Meters)
            * Math.PI;

    logShooterTanSpeed.info(shooterTangentialVel);

    double horizShooterTangentialVel =
        shooterTangentialVel * Math.cos(shooterAngleOfShot.getRadians());

    double poseOfShotRotation = robotPoseOfShot.getRotation().getRadians();
    Translation3d gamepieceVel =
        new Translation3d(
            -robotVelOfShot.getX() + horizShooterTangentialVel * Math.cos(poseOfShotRotation),
            -robotVelOfShot.getY() + horizShooterTangentialVel * Math.sin(poseOfShotRotation),
            shooterTangentialVel * Math.sin(shooterAngleOfShot.getRadians()));
    double gamepieceLinearVel = gamepieceVel.getDistance(new Translation3d());
    Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(gamepieceVel.getY(), gamepieceVel.getX()));
    Rotation2d pitch =
        Rotation2d.fromRadians(
            Math.atan2(gamepieceVel.getZ(), Math.hypot(gamepieceVel.getX(), gamepieceVel.getY())));

    showingPath = !(shootingTime * gamepieceLinearVel <= dist);
    showPath = showingPath && !prevShowingPath;
    if (!poses.isEmpty()) {
      poses.set(
          0, new Pair<Pose3d, Double>(new Pose3d(0.0, 0.0, -1000.0, new Rotation3d()), 10000000.0));
    }
    if (shootingGamepiece) {
      if (shootingTime * gamepieceLinearVel <= dist) {
        if (initial) {
          poses.add(
              0,
              new Pair<Pose3d, Double>(
                  new Pose3d(
                      new Translation3d(
                              shootingTime * gamepieceLinearVel,
                              new Rotation3d(0.0, -pitch.getRadians(), yaw.getRadians() + Math.PI))
                          .plus(GeometryUtil.translation2dTo3d(robotPoseOfShot.getTranslation())),
                      new Rotation3d(0.0, shooterAngleOfShot.getRadians(), poseOfShotRotation)),
                  Timer.getFPGATimestamp()));
          initial = false;
        }
        poses.set(
            0,
            new Pair<Pose3d, Double>(
                new Pose3d(
                    new Translation3d(
                            shootingTime * gamepieceLinearVel,
                            new Rotation3d(0.0, -pitch.getRadians(), yaw.getRadians() + Math.PI))
                        .plus(GeometryUtil.translation2dTo3d(robotPoseOfShot.getTranslation())),
                    new Rotation3d(0.0, shooterAngleOfShot.getRadians(), poseOfShotRotation)),
                Timer.getFPGATimestamp()));
      } else {
        if (showPath) {
          for (int i = 0; i < (int) (dist / gamepieceSpacing) + 3; i++) {
            poses.add(
                new Pair<Pose3d, Double>(
                    new Pose3d(
                        new Translation3d(
                                gamepieceSpacing * i,
                                new Rotation3d(
                                    0.0, -pitch.getRadians(), yaw.getRadians() + Math.PI))
                            .plus(GeometryUtil.translation2dTo3d(robotPoseOfShot.getTranslation())),
                        new Rotation3d(0.0, shooterAngleOfShot.getRadians(), poseOfShotRotation)),
                    Timer.getFPGATimestamp()));
          }
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
    logGamepieceTraj.info(loggedPoses);
  }
}
