package frc.lib.team2930;

import frc.lib.team6328.LoggedTunableNumber;

public class TunableNumberGroup {
  private final String root;

  public TunableNumberGroup(String root) {
    this.root = root;
  }

  public LoggedTunableNumber build(String name, double defaultValue) {
    return new LoggedTunableNumber(root + "/" + name, defaultValue);
  }
}
