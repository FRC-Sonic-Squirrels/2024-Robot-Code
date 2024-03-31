package frc.robot.subsystems.vision;

public enum VisionResultStatus {
  VISION_DISABLED(false),
  NOT_A_NEW_RESULT(false),
  PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY(false),
  NO_TARGETS_VISIBLE(false),
  TOO_FAR_FROM_EXISTING_ESTIMATE(false),
  INVALID_POSE_OUTSIDE_FIELD(false),
  INVALID_TAG_AMBIGUITY_TOO_HIGH(false),
  INVALID_TAG(false),
  SUCCESSFUL_MULTI_TAG(true),
  SUCCESSFUL_SINGLE_TAG(true),
  NOT_CLOSE_ENOUGH_TO_GYRO_ROTATION(false),
  Z_HEIGHT_BAD(false),
  PITCH_OR_ROLL_BAD(false),
  INIT(false),
  UNKNOWN(false);

  public final boolean success;

  private VisionResultStatus(boolean success) {
    this.success = success;
  }
}
