package frc.lib.team2930;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Arrays;

public class LoggerGroup {
  @FunctionalInterface
  public interface EntrySupplier<T extends LoggerEntry> {
    T apply(String name, int updateFrequency);
  }

  private static double currentTimestmap;
  public static final LoggerGroup root;

  private final String path;
  private LoggerGroup[] groups = new LoggerGroup[0];
  private LoggerEntry[] entries = new LoggerEntry[0];

  static {
    root = new LoggerGroup("");
  }

  public static void periodic() {
    currentTimestmap = Utils.getCurrentTimeSeconds();
  }

  static double shouldRefresh(double lastRefresh, double updateFrequency) {
    if (lastRefresh + updateFrequency <= currentTimestmap) {
      return currentTimestmap + updateFrequency;
    }

    return Double.NaN;
  }

  private LoggerGroup(String path) {
    this.path = path;
  }

  // -- //

  public LoggerEntry.Text buildString(String name) {
    return buildString(name, 50);
  }

  public LoggerEntry.Text buildString(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Text.class, LoggerEntry.Text::new);
  }

  // -- //

  public <E extends Enum<E>> LoggerEntry.EnumValue<E> buildEnum(String name) {
    return buildEnum(name, 50);
  }

  public <E extends Enum<E>> LoggerEntry.EnumValue<E> buildEnum(
      String name, int updateFrequencyInSeconds) {
    //noinspection unchecked
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.EnumValue.class, LoggerEntry.EnumValue::new);
  }

  // -- //

  public LoggerEntry.Bool buildBoolean(String name) {
    return buildBoolean(name, 50);
  }

  public LoggerEntry.Bool buildBoolean(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Bool.class, LoggerEntry.Bool::new);
  }

  // -- //

  public LoggerEntry.Integer buildInteger(String name) {
    return buildInteger(name, 50);
  }

  public LoggerEntry.Integer buildInteger(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Integer.class, LoggerEntry.Integer::new);
  }

  public LoggerEntry.IntegerArray buildIntegerArray(String name) {
    return buildIntegerArray(name, 50);
  }

  public LoggerEntry.IntegerArray buildIntegerArray(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name,
        updateFrequencyInSeconds,
        LoggerEntry.IntegerArray.class,
        LoggerEntry.IntegerArray::new);
  }

  // -- //

  public LoggerEntry.Decimal buildDecimal(String name) {
    return buildDecimal(name, 50);
  }

  public LoggerEntry.Decimal buildDecimal(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Decimal.class, LoggerEntry.Decimal::new);
  }

  public LoggerEntry.DecimalArray buildDecimalArray(String name) {
    return buildDecimalArray(name, 50);
  }

  public LoggerEntry.DecimalArray buildDecimalArray(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name,
        updateFrequencyInSeconds,
        LoggerEntry.DecimalArray.class,
        LoggerEntry.DecimalArray::new);
  }

  // -- //

  public LoggerEntry.Mechanism buildMechanism2d(String name) {
    return buildMechanism2d(name, 50);
  }

  public LoggerEntry.Mechanism buildMechanism2d(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Mechanism.class, LoggerEntry.Mechanism::new);
  }

  // -- //

  public <T extends StructSerializable> LoggerEntry.Struct<T> buildStruct(
      Class<T> clz, String name) {
    return buildStruct(clz, name, 50);
  }

  public <T extends StructSerializable> LoggerEntry.Struct<T> buildStruct(
      Class<T> clz, String name, int updateFrequencyInSeconds) {
    //noinspection unchecked
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Struct.class, LoggerEntry.Struct::new);
  }

  // -- //

  public <T extends StructSerializable> LoggerEntry.StructArray<T> buildStructArray(
      Class<T> clz, String name) {
    return buildStructArray(clz, name, 50);
  }

  public <T extends StructSerializable> LoggerEntry.StructArray<T> buildStructArray(
      Class<T> clz, String name, int updateFrequencyInSeconds) {
    //noinspection unchecked
    return buildInner(
        name,
        updateFrequencyInSeconds,
        LoggerEntry.StructArray.class,
        LoggerEntry.StructArray::new);
  }

  public LoggerEntry.ByteArray buildBytes(String name) {
    return buildBytes(name, 50);
  }

  public LoggerEntry.ByteArray buildBytes(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.ByteArray.class, LoggerEntry.ByteArray::new);
  }

  // -- //

  // -- //

  private synchronized <T extends LoggerEntry> T buildInner(
      String name, int updateFrequencyInSeconds, Class<T> clz, EntrySupplier<T> supplier) {
    var ptr = this;
    int pos = name.indexOf('/');
    if (pos >= 0) {
      var path = name.substring(0, pos);
      var subName = name.substring(pos + 1);

      if (!path.isEmpty()) {
        ptr = ptr.subgroup(path);
      }

      return ptr.buildInner(subName, updateFrequencyInSeconds, clz, supplier);
    }

    var key = path.isEmpty() ? name : path + "/" + name;
    for (var entry : entries) {
      if (entry.key.equals(key)) {
        return clz.cast(entry);
      }
    }

    var newEntry = supplier.apply(key, updateFrequencyInSeconds);
    entries = Arrays.copyOf(entries, entries.length + 1);
    entries[entries.length - 1] = newEntry;
    return newEntry;
  }

  public static LoggerGroup build(String name) {
    return root.subgroup(name);
  }

  public synchronized LoggerGroup subgroup(String name) {
    var ptr = this;
    int pos = name.indexOf('/');
    if (pos >= 0) {
      var path = name.substring(0, pos);
      var subName = name.substring(pos + 1);

      if (!path.isEmpty()) {
        ptr = ptr.subgroup(path);
      }

      return ptr.subgroup(subName);
    }

    var fullPath = path.isEmpty() ? name : path + "/" + name;
    for (var group : groups) {
      if (group.path.equals(fullPath)) {
        return group;
      }
    }

    var newGroup = new LoggerGroup(fullPath);
    groups = Arrays.copyOf(groups, groups.length + 1);
    groups[groups.length - 1] = newGroup;
    return newGroup;
  }
}
