package frc.lib.team2930;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class AutoLock {
  private final Lock state = new ReentrantLock();

  public class Handler implements AutoCloseable {
    private Handler() {
      state.lock();
    }

    @Override
    public void close() {
      state.unlock();
    }
  }

  public Handler lock() {
    return new Handler();
  }
}
