package frc.lib.team2930;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

public abstract class LoggerEntry {
  public final String key;
  public final double updateFrequency;
  public double lastRefresh;
  protected GenericPublisher publisher;
  boolean notFirstValue;
  boolean changed;
  int dataLogId = LoggerGroup.dataLogIdNotInitialized;

  protected LoggerEntry(String key, int updateFrequencyInSeconds) {
    this.key = key;
    this.updateFrequency = 1.0 / updateFrequencyInSeconds;
  }

  void registerPublisher() {
    String nt4Type = getNT4Type();
    if (nt4Type != null) {
      this.publisher =
          LoggerGroup.getNetworkTableTopic(this)
              .genericPublish(nt4Type, PubSubOption.sendAll(true));
    }
  }

  @Override
  public String toString() {
    return key;
  }

  public abstract String getNT4Type();

  public abstract String getWpiLogType();

  public abstract void publish();

  protected int getDataLogId() {
    if (dataLogId == LoggerGroup.dataLogIdNotInitialized) {
      dataLogId = LoggerGroup.getDataLogTopic(this);
    }

    return dataLogId;
  }

  // ############

  public static class Condition {
    public State build() {
      return new State();
    }

    public class State {
      private double timeStart = Double.NaN;
      private double timeTrigger = Double.NaN;
      private boolean state;

      public void start() {
        if (Double.isNaN(timeStart)) {
          timeStart = LoggerGroup.getCurrentTimestmap();
        }
      }

      public void set(boolean state) {
        if (state) {
          if (Double.isNaN(timeTrigger)) {
            timeTrigger = LoggerGroup.getCurrentTimestmap();
          }
        }

        this.state = state;
      }

      public boolean get() {
        return state;
      }

      public void log() {
        double delay;

        if (Double.isNaN(timeStart)) {
          delay = 0;
        } else if (Double.isNaN(timeTrigger)) {
          delay = LoggerGroup.getCurrentTimestmap() - timeStart;
        } else {
          delay = timeTrigger - timeStart;
        }

        log_condition.info(state);
        log_conditionDelay.info(delay);
      }
    }

    public final LoggerEntry.Bool log_condition;
    public final LoggerEntry.Decimal log_conditionDelay;

    public Condition(LoggerGroup group, String prefix) {
      log_condition = group.buildBoolean(prefix);
      log_conditionDelay = group.buildDecimal(prefix + "_Latency");
    }
  }

  // ############

  public static class Text extends LoggerEntry {
    private String valuePrevious;
    private String value;

    Text(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "string";
    }

    @Override
    public String getWpiLogType() {
      return "string";
    }

    @Override
    public void publish() {
      LoggerGroup.emit(publisher, value, getDataLogId());
      valuePrevious = value;
    }

    public void info(String value) {
      this.value = value;
      this.changed = !Objects.equals(value, valuePrevious) || !notFirstValue;
    }
  }

  public static class TextArray extends LoggerEntry {
    private String[] valuePrevious;
    private String[] value;

    TextArray(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "string[]";
    }

    @Override
    public String getWpiLogType() {
      return "string[]";
    }

    @Override
    public void publish() {
      var valueCloned = value != null ? Arrays.copyOf(value, value.length) : null;
      LoggerGroup.emit(publisher, valueCloned, getDataLogId());
      valuePrevious = valueCloned;
    }

    public void info(String... value) {
      this.value = value;
      this.changed = !Arrays.equals(value, valuePrevious) || !notFirstValue;
    }
  }

  public static class EnumValue<E extends Enum<E>> extends LoggerEntry {
    private E valuePrevious;
    private E value;

    EnumValue(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "string";
    }

    @Override
    public String getWpiLogType() {
      return "string";
    }

    @Override
    public void publish() {
      if (value != null) {
        LoggerGroup.emit(publisher, value.name(), getDataLogId());
      }

      valuePrevious = value;
    }

    public void info(E value) {
      this.value = value;
      this.changed = value != valuePrevious || !notFirstValue;
    }
  }

  public static class Bool extends LoggerEntry {
    private boolean valuePrevious;
    private boolean value;

    Bool(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "boolean";
    }

    @Override
    public String getWpiLogType() {
      return "boolean";
    }

    @Override
    public void publish() {
      LoggerGroup.emit(publisher, value, getDataLogId());
      valuePrevious = value;
    }

    public void info(boolean value) {
      this.value = value;
      this.changed = value != valuePrevious || !notFirstValue;
    }
  }

  public static class BoolArray extends LoggerEntry {
    private boolean[] valuePrevious;
    private boolean[] value;

    BoolArray(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "boolean[]";
    }

    @Override
    public String getWpiLogType() {
      return "boolean[]";
    }

    @Override
    public void publish() {
      LoggerGroup.emit(publisher, value, getDataLogId());
      valuePrevious = value;
    }

    public void info(boolean... value) {
      this.value = value;
      this.changed = value != valuePrevious || !notFirstValue;
    }
  }

  public static class Integer extends LoggerEntry {
    private long valuePrevious;
    private long value;

    Integer(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "int";
    }

    @Override
    public String getWpiLogType() {
      return "int64";
    }

    @Override
    public void publish() {
      LoggerGroup.emit(publisher, value, getDataLogId());
      valuePrevious = value;
    }

    public void info(long value) {
      this.value = value;
      this.changed = value != valuePrevious || !notFirstValue;
    }
  }

  public static class IntegerArray extends LoggerEntry {
    private long[] valuePrevious;
    private long[] value;

    IntegerArray(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "int[]";
    }

    @Override
    public String getWpiLogType() {
      return "int64[]";
    }

    @Override
    public void publish() {
      var valueCloned = value != null ? Arrays.copyOf(value, value.length) : null;
      LoggerGroup.emit(publisher, valueCloned, getDataLogId());
      valuePrevious = valueCloned;
    }

    public void info(long... value) {
      this.value = value;
      this.changed = !Arrays.equals(value, valuePrevious) || !notFirstValue;
    }
  }

  public static class Decimal extends LoggerEntry {
    private double valuePrevious;
    private double value;

    Decimal(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "double";
    }

    @Override
    public String getWpiLogType() {
      return "double";
    }

    @Override
    public void publish() {
      LoggerGroup.emit(publisher, value, getDataLogId());
      valuePrevious = value;
    }

    public void info(double value) {
      this.value = value;
      this.changed = value != valuePrevious || !notFirstValue;
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

    @Override
    public String getNT4Type() {
      return "double[]";
    }

    @Override
    public String getWpiLogType() {
      return "double[]";
    }

    @Override
    public void publish() {
      var valueCloned = value != null ? Arrays.copyOf(value, value.length) : null;
      LoggerGroup.emit(publisher, valueCloned, getDataLogId());
      valuePrevious = valueCloned;
    }

    public void info(double... value) {
      this.value = value;
      this.changed = !Arrays.equals(value, valuePrevious) || !notFirstValue;
    }
  }

  public static class DecimalFloat extends LoggerEntry {
    private float valuePrevious;
    private float value;

    DecimalFloat(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "float";
    }

    @Override
    public String getWpiLogType() {
      return "float";
    }

    @Override
    public void publish() {
      LoggerGroup.emit(publisher, value, getDataLogId());
      valuePrevious = value;
    }

    public void info(float value) {
      this.value = value;
      this.changed = value != valuePrevious || !notFirstValue;
    }
  }

  public static class DecimalFloatArray extends LoggerEntry {
    private float[] valuePrevious;
    private float[] value;

    DecimalFloatArray(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "float[]";
    }

    @Override
    public String getWpiLogType() {
      return "float[]";
    }

    @Override
    public void publish() {
      var valueCloned = value != null ? Arrays.copyOf(value, value.length) : null;
      LoggerGroup.emit(publisher, valueCloned, getDataLogId());
      valuePrevious = valueCloned;
    }

    public void info(float... value) {
      this.value = value;
      this.changed = !Arrays.equals(value, valuePrevious) || !notFirstValue;
    }
  }

  public static class ByteArray extends LoggerEntry {
    private byte[] valuePrevious;
    private byte[] value;

    ByteArray(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return "raw";
    }

    @Override
    public String getWpiLogType() {
      return "raw";
    }

    @Override
    public void publish() {
      var valueCloned = value != null ? Arrays.copyOf(value, value.length) : null;
      LoggerGroup.emit(publisher, valueCloned, getDataLogId());
      valuePrevious = valueCloned;
    }

    public void info(byte[] value) {
      this.value = value;
      this.changed = !Arrays.equals(value, valuePrevious) || !notFirstValue;
    }
  }

  public static class Mechanism extends LoggerEntry {
    private Mechanism2d valuePrevious;
    private Mechanism2d value;

    Mechanism(String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);
    }

    @Override
    public String getNT4Type() {
      return null;
    }

    @Override
    public String getWpiLogType() {
      return null;
    }

    @Override
    public void publish() {
      // This uses internal implementation details of Mechanism2d, keeping it on AdvantageKit
      // logging.
      Logger.recordOutput(key, value);
      valuePrevious = value;
    }

    public void info(Mechanism2d value) {
      this.value = value;
      this.changed = true;
    }
  }

  public static class Struct<T extends StructSerializable> extends LoggerEntry {
    private final edu.wpi.first.util.struct.Struct<T> struct;
    private final StructBuffer<T> structBuffer;
    private T valuePrevious;
    private T value;

    Struct(Class<T> clz, String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);

      struct = LoggerGroup.findStructType(clz);
      structBuffer = struct != null ? StructBuffer.create(struct) : null;
    }

    @Override
    public String getNT4Type() {
      if (struct != null) {
        return struct.getTypeString();
      }

      return "raw";
    }

    @Override
    public String getWpiLogType() {
      return getNT4Type();
    }

    @Override
    public void publish() {
      if (structBuffer != null) {
        ByteBuffer bb = structBuffer.write(value);
        byte[] array = new byte[bb.position()];
        bb.position(0);
        bb.get(array);

        //        Logger.recordOutput(key, value);
        LoggerGroup.emit(publisher, array, getDataLogId());
        valuePrevious = value;
      }
    }

    public void info(T value) {
      this.value = value;
      this.changed = true;
    }
  }

  public static class StructArray<T extends StructSerializable> extends LoggerEntry {
    private final edu.wpi.first.util.struct.Struct<T> struct;
    private final StructBuffer<T> structBuffer;
    private T[] valuePrevious;
    private T[] value;

    StructArray(Class<T> clz, String key, int updateFrequencyInSeconds) {
      super(key, updateFrequencyInSeconds);

      struct = LoggerGroup.findStructType(clz);
      structBuffer = struct != null ? StructBuffer.create(struct) : null;
    }

    @Override
    public String getNT4Type() {
      if (struct != null) {
        return struct.getTypeString() + "[]";
      }

      return "raw";
    }

    @Override
    public String getWpiLogType() {
      return getNT4Type();
    }

    @Override
    public void publish() {
      if (structBuffer != null) {
        ByteBuffer bb = structBuffer.writeArray(value);
        byte[] array = new byte[bb.position()];
        bb.position(0);
        bb.get(array);

        //        Logger.recordOutput(key, value);
        LoggerGroup.emit(publisher, array, getDataLogId());
        valuePrevious = value;
      }
    }

    public void info(T[] value) {
      this.value = value;
      this.changed = true;
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

  void publishIfNeeded() {
    if (shouldNotRefresh()) return;

    if (changed) {
      changed = false;
      notFirstValue = true;
      publish();
    }
  }
}
