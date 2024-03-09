package frc.robot.subsystems.vision;

public enum VisionResultStatus {
  VISION_DISABLED(""),
  NOT_A_NEW_RESULT(""),
  PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY(""),
  NO_TARGETS_VISIBLE(""),
  TOO_FAR_FROM_EXISTING_ESTIMATE(""),
  INVALID_POSE_OUTSIDE_FIELD(""),
  INVALID_TAG_AMBIGUITY_TOO_HIGH(""),
  INVALID_TAG(""),
  SUCCESSFUL_MULTI_TAG(""),
  SUCCESSFUL_SINGLE_TAG(""),
  SUCCESSFUL_SINGLE_TAG_BECAUSE_MULTI_TAG_FALLBACK(""),
  NOT_CLOSE_ENOUGH_TO_GYRO_ROTATION(""),
  Z_HEIGHT_BAD(""),
  PITCH_OR_ROLL_BAD(""),
  INIT("start of vision code"),

  UNKNOWN("UNKNOWN");

  public final String additionalInfo;

  private VisionResultStatus(String additionalInfo) {
    this.additionalInfo = additionalInfo;
  }
}
