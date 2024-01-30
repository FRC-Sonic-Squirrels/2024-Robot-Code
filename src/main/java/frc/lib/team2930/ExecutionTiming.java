package frc.lib.team2930;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class ExecutionTiming implements AutoCloseable {
  private static final String ROOT_TABLE = "ExecutionTiming/";

  private final String context;
  private final double startTime;

  public ExecutionTiming(String context) {
    this.context = context;
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void close() {
    var endTime = Timer.getFPGATimestamp();

    Logger.recordOutput(ROOT_TABLE + context, endTime - startTime);
  }
}
