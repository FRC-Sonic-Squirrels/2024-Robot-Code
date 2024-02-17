package frc.robot.autonomous;

public class AutoState {
  public AutoStates state;
  public double timestamp;

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
