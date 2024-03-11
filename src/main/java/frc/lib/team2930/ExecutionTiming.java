package frc.lib.team2930;

import org.littletonrobotics.junction.Logger;

public class ExecutionTiming implements AutoCloseable {
  private static final String ROOT_TABLE = "ExecutionTiming/";

  private final String context;
  private final long startTime;

  public ExecutionTiming(String context) {
    this.context = context;
    startTime = Logger.getRealTimestamp();
  }

  @Override
  public void close() {
    var endTime = Logger.getRealTimestamp();

    Logger.recordOutput(ROOT_TABLE + context, endTime - startTime);
  }
}
