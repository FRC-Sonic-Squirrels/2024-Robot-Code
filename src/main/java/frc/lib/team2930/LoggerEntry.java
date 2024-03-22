package frc.lib.team2930;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.Logger;

public abstract class LoggerEntry {
  private static final Object g_lock = new Object();

  public final String key;
  public final double updateFrequency;
  public double lastRefresh;
  boolean changed;

  protected LoggerEntry(String key, int updateFrequencyInSeconds) {
    this.key = key;
    this.updateFrequency = 1.0 / updateFrequencyInSeconds;
  }

  public static class Text extends LoggerEntry {
    private String valuePrevious;
    private String value;

    Text(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(String value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }
  }

  public static class EnumValue<E extends Enum<E>> extends LoggerEntry {
    private E valuePrevious;
    private E value;

    EnumValue(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(E value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }
  }

  public static class Bool extends LoggerEntry {
    private boolean valuePrevious;
    private boolean value;

    Bool(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(boolean value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }
  }

  public static class Integer extends LoggerEntry {
    private long valuePrevious;
    private long value;

    Integer(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(long value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }
  }

  public static class IntegerArray extends LoggerEntry {
    private long[] valuePrevious;
    private long[] value;

    IntegerArray(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(long... value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }
  }

  public static class Decimal extends LoggerEntry {
    private double valuePrevious;
    private double value;

    Decimal(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(double value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }

    public void info(Rotation2d value) {
      info(value.getDegrees());
    }
  }

  public static class DecimalArray extends LoggerEntry {
    private double[] valuePrevious;
    private double[] value;

    DecimalArray(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(double... value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }
  }

  public static class ByteArray extends LoggerEntry {
    private byte[] valuePrevious;
    private byte[] value;

    ByteArray(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(byte[] value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }
  }

  public static class Mechanism extends LoggerEntry {
    private Mechanism2d valuePrevious;
    private Mechanism2d value;

    Mechanism(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(Mechanism2d value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }
  }

  public static class Struct<T extends StructSerializable> extends LoggerEntry {
    private T valuePrevious;
    private T value;

    Struct(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(T value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }
  }

  public static class StructArray<T extends StructSerializable> extends LoggerEntry {
    private T[] valuePrevious;
    private T[] value;

    StructArray(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    public void info(T[] value) {
      if (shouldNotRefresh()) return;

      this.value = value;
      this.changed = true;

      synchronized (g_lock) {
        Logger.recordOutput(key, value);
      }
    }
  }

  protected boolean shouldNotRefresh() {
    var updated = LoggerGroup.shouldRefresh(lastRefresh, updateFrequency);
    if (Double.isFinite(updated)) {
      lastRefresh = updated;
      return false;
    }

    return true;
  }
}
