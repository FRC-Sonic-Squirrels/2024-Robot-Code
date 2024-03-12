package frc.lib.team2930;

import java.util.HashMap;

public class LoggerGroup {
  private final String root;
  private final HashMap<String, LoggerGroup> groups = new HashMap<>();
  private final HashMap<String, LoggerEntry> entries = new HashMap<>();

  public LoggerGroup(String root) {
    this.root = root;
  }

  public synchronized LoggerEntry build(String name) {
    var entry = entries.get(name);
    if (entry == null) {
      entry = new LoggerEntry(root + "/" + name);
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
