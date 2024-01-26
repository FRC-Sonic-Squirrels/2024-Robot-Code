// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotState {
  private static RobotState instance = null;
  private ScoringMode scoringMode = ScoringMode.NONE;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }

    return instance;
  }

  private RobotState() {}

  public ScoringMode getScoringMode() {
    return scoringMode;
  }

  public void setScoringMode(ScoringMode scoringMode) {
    this.scoringMode = scoringMode;
  }

  public static enum IntakeMode {
    INTAKE,
    STOW;
  }

  public static enum ScoringMode {
    NONE,
    SPEAKER,
    AMP;
  }
}
