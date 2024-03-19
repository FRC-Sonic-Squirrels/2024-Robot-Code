package frc.lib.team2930;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import us.hebi.quickbuf.ProtoMessage;

public class LoggerEntry {
  private static final Object g_lock = new Object();

  public final String key;
  public final double updateFrequency;
  public double lastRefresh;

  public LoggerEntry(String key) {
    this(key, 50);
  }

  public LoggerEntry(String key, int updateFrequencyInSeconds) {
    this.key = key;
    this.updateFrequency = 1.0 / updateFrequencyInSeconds;
  }

  public void info(byte[] value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(boolean value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(int value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(long value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(float value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(double value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(String value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public <E extends Enum<E>> void info(E value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public <U extends Unit<U>> void info(Measure<U> value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(boolean[] value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(int[] value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(long[] value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(float[] value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(double[] value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(String[] value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public <T> void info(Struct<T> struct, T value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, struct, value);
    }
  }

  public <T> void info(Struct<T> struct, T... value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, struct, value);
    }
  }

  public <T, MessageType extends ProtoMessage<?>> void info(
      Protobuf<T, MessageType> proto, T value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, proto, value);
    }
  }

  public <T extends WPISerializable> void info(T value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public <T extends StructSerializable> void info(T... value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(Mechanism2d value) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.recordOutput(key, value);
    }
  }

  public void info(Rotation2d value) {
    info(value.getDegrees());
  }

  public void info(LoggableInputs inputs) {
    if (shouldNotRefresh()) return;

    synchronized (g_lock) {
      Logger.processInputs(key, inputs);
    }
  }

  private boolean shouldNotRefresh() {
    var updated = LoggerGroup.shouldRefresh(lastRefresh, updateFrequency);
    if (Double.isFinite(updated)) {
      lastRefresh = updated;
      return false;
    }

    return true;
  }
}
