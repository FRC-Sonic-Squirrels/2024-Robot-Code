package frc.robot.autonomous1;

import edu.wpi.first.math.controller.PIDController;

public class AutoConstants {

  // TODO: TUNE VALUES
  public static final PIDController AUTO_X_PID = new PIDController(10, 0, 0);

  public static final PIDController AUTO_Y_PID = new PIDController(10, 0, 0);
  public static final PIDController AUTO_THETA_PID = new PIDController(10, 0, 0);
}
