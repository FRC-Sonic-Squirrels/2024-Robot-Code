package frc.lib.constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class SwerveModuleConstants {

  public static class MK4I {
    public static final InvertedValue DEFAULT_DRIVE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final InvertedValue DEFAULT_STEER_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

    public static final double LEVEL_2_GEARING_DRIVE_GEAR_RATIO =
        (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

    public static final double LEVEL_3_GEARING_DRIVE_GEAR_RATIO_PLUS_SPEED_KIT =
        (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);

    public static final double GEARING_TURN_GEAR_RATIO = 150.0 / 7.0;
  }
}
