package frc.lib.team2930;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class AutoLock implements AutoCloseable {
  private final Lock state = new ReentrantLock();
  private final int waitThreshold;
  private final LoggerEntry log_averageTime;
  private final LoggerEntry log_timeViolation;
  private final LoggerEntry log_count;
  private final LoggerEntry log_countViolations;
  private int counter;
  private int counterViolations;
  private long totalTime;

  public AutoLock(String name, int waitThreshold) {
    this.waitThreshold = waitThreshold;
    log_averageTime = new LoggerEntry("AutoLock/" + name + "/averageTime");
    log_timeViolation = new LoggerEntry("AutoLock/" + name + "/timeViolation");
    log_count = new LoggerEntry("AutoLock/" + name + "/count");
    log_countViolations = new LoggerEntry("AutoLock/" + name + "/countViolations");
  }

  public AutoLock lock() {
    var startTime = Logger.getRealTimestamp();
    state.lock();
    counter++;
    var endTime = Logger.getRealTimestamp();
    var time = endTime - startTime;
    totalTime += time;
    if (time > waitThreshold) {
      counterViolations++;

      log_averageTime.info((double) totalTime / counter);
      log_count.info(counter);

      log_timeViolation.info(time);
      log_countViolations.info(counterViolations);
    }

    if (counter % 1024 == 0) {
      log_count.info(counter);
      log_averageTime.info((double) totalTime / counter);
    }

    return this;
  }

  public void close() {
    state.unlock();
  }
}
