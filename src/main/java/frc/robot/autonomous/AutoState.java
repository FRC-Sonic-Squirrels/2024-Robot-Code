package frc.robot.autonomous;

public class AutoState {
  public AutoStates state = AutoStates.FOLLOW_PATH;
  public double timestamp = 0.0;

  public AutoState(AutoStates state, double timestamp) {
    this.state = state;
    this.timestamp = timestamp;
  }

  public enum AutoStates {
    FOLLOW_PATH,
    SHOOTING,
    INTAKING
  }
}
