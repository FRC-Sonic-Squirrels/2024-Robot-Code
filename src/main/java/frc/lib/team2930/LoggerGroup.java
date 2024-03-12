package frc.lib.team2930;

import com.ctre.phoenix6.Utils;
import java.util.HashMap;

public class LoggerGroup {
  private static double currentTimestmap;

  private final String root;
  private final HashMap<String, LoggerGroup> groups = new HashMap<>();
  private final HashMap<String, LoggerEntry> entries = new HashMap<>();

  public static void periodic() {
    currentTimestmap = Utils.getCurrentTimeSeconds();
  }

  static double shouldRefresh(double lastRefresh, double updateFrequency) {
    if (lastRefresh + updateFrequency <= currentTimestmap) {
      return currentTimestmap + updateFrequency;
    }

    return Double.NaN;
  }

  public LoggerGroup(String root) {
    this.root = root;
  }

  public LoggerEntry build(String name) {
    return build(name, 50);
  }

  public synchronized LoggerEntry build(String name, int updateFrequencyInSeconds) {
    var entry = entries.get(name);
    if (entry == null) {
      entry = new LoggerEntry(root + "/" + name, updateFrequencyInSeconds);
      entries.put(name, entry);
    }
    return entry;
  }

  public synchronized LoggerGroup subgroup(String name) {
    var group = groups.get(name);
    if (group == null) {
      group = new LoggerGroup(root + "/" + name);
      groups.put(name, group);
    }
    return group;
  }
}
