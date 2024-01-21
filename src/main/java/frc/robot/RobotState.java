// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotState {
  private static RobotState instance = null;
  private IntakeMode intakeMode = IntakeMode.INTAKE;
  private ScoringMode shootMode = ScoringMode.SPEAKER;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }

    return instance;
  }

  private RobotState() {}

  public IntakeMode getIntakeMode() {
    return intakeMode;
  }

  public void setIntakeMode(IntakeMode intakeMode) {
    this.intakeMode = intakeMode;
  }

  public ScoringMode getScoringMode() {
    return shootMode;
  }

  public void setScoringMode(ScoringMode shootMode) {
    this.shootMode = shootMode;
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
