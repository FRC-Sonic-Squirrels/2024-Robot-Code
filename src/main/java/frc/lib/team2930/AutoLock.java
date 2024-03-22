package frc.lib.team2930;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class AutoLock implements AutoCloseable {
  private final Lock state = new ReentrantLock();
  private final int waitThreshold;
  private final LoggerEntry.Decimal log_averageTime;
  private final LoggerEntry.Integer log_timeViolation;
  private final LoggerEntry.Integer log_count;
  private final LoggerEntry.Integer log_countViolations;
  private int counter;
  private int counterViolations;
  private long totalTime;

  public AutoLock(String name, int waitThreshold) {
    this.waitThreshold = waitThreshold;

    var group = LoggerGroup.build("AutoLock");
    var subgroup = group.subgroup(name);

    log_averageTime = subgroup.buildDecimal("averageTime");
    log_timeViolation = subgroup.buildInteger("timeViolation");
    log_count = subgroup.buildInteger("count");
    log_countViolations = subgroup.buildInteger("countViolations");
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
