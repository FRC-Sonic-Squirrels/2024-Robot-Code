package frc.robot.autonomous.records;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public record TargetGP(Translation2d targetGP, Pose2d prepForTargetGP) {

  private static double CENTER_LINE = Constants.FieldConstants.FIELD_LENGTH / 2.0;
  private static double DELTA_BETWEEN_CENTER_LINE_NOTES = 1.68;
  private static double GP_1_Y = 7.47;
  private static double GP_1_PREP_POSE_Y = 6.63;
  private static double PREP_POSE_X = CENTER_LINE - 1.27;

  private static double CLOSE_GP_X = 2.89;
  private static double CLOSE_GP_1_Y = 7.02;
  private static double CLOSE_GP_DELTA = 1.46;

  private static double CLOSE_GP_X_FROM_RUSH = 2.89;

  // FIXME: Remember to flip this for alliance
  private static Rotation2d CENTER_LINE_PREP_ROTATION = Rotation2d.fromDegrees(0.0);
  private static Rotation2d CLOSE_GP_FROM_RUSH_ROTATION = Rotation2d.fromDegrees(180.0);

  public static TargetGP GP_1 =
      new TargetGP(
          new Translation2d(CENTER_LINE, GP_1_Y),
          new Pose2d(PREP_POSE_X, GP_1_PREP_POSE_Y, CENTER_LINE_PREP_ROTATION));

  public static TargetGP GP_2 =
      new TargetGP(
          new Translation2d(CENTER_LINE, GP_1_Y - DELTA_BETWEEN_CENTER_LINE_NOTES * 1.0),
          new Pose2d(
              PREP_POSE_X,
              GP_1_PREP_POSE_Y - DELTA_BETWEEN_CENTER_LINE_NOTES * 1.0,
              CENTER_LINE_PREP_ROTATION));
  public static TargetGP GP_3 =
      new TargetGP(
          new Translation2d(CENTER_LINE, GP_1_Y - DELTA_BETWEEN_CENTER_LINE_NOTES * 2.0),
          new Pose2d(
              PREP_POSE_X,
              GP_1_PREP_POSE_Y - DELTA_BETWEEN_CENTER_LINE_NOTES * 2.0,
              CENTER_LINE_PREP_ROTATION));
  public static TargetGP GP_4 =
      new TargetGP(
          new Translation2d(CENTER_LINE, GP_1_Y - DELTA_BETWEEN_CENTER_LINE_NOTES * 3.0),
          new Pose2d(
              PREP_POSE_X,
              GP_1_PREP_POSE_Y - DELTA_BETWEEN_CENTER_LINE_NOTES * 3.0,
              CENTER_LINE_PREP_ROTATION));
  public static TargetGP GP_5 =
      new TargetGP(
          new Translation2d(CENTER_LINE, GP_1_Y - DELTA_BETWEEN_CENTER_LINE_NOTES * 4.0),
          new Pose2d(
              PREP_POSE_X,
              GP_1_PREP_POSE_Y - DELTA_BETWEEN_CENTER_LINE_NOTES * 4.0,
              CENTER_LINE_PREP_ROTATION));

  public static TargetGP CLOSE_GP_1 =
      new TargetGP(new Translation2d(CLOSE_GP_X, CLOSE_GP_1_Y), null);
  public static TargetGP CLOSE_GP_2 =
      new TargetGP(new Translation2d(CLOSE_GP_X, CLOSE_GP_1_Y - CLOSE_GP_DELTA * 1.0), null);
  public static TargetGP CLOSE_GP_3 =
      new TargetGP(new Translation2d(CLOSE_GP_X, CLOSE_GP_1_Y - CLOSE_GP_DELTA * 2.0), null);

  public static TargetGP CLOSE_GP_1_FROM_RUSH =
      new TargetGP(
          CLOSE_GP_1.targetGP,
          new Pose2d(CLOSE_GP_X_FROM_RUSH, CLOSE_GP_1_Y, CLOSE_GP_FROM_RUSH_ROTATION));

  public static TargetGP CLOSE_GP_2_FROM_RUSH =
      new TargetGP(
          CLOSE_GP_1.targetGP,
          new Pose2d(
              CLOSE_GP_X_FROM_RUSH,
              CLOSE_GP_1_Y - DELTA_BETWEEN_CENTER_LINE_NOTES * 1.0,
              CLOSE_GP_FROM_RUSH_ROTATION));

  public static TargetGP SOURCE_PLOP = new TargetGP(new Translation2d(0.85, 1.48), null);

  // close coming from rush
}
