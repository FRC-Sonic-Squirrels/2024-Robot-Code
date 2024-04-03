package frc.robot.autonomous;

public record PathDescriptor(
    String intakingTraj, String shootingTraj, boolean useVision, boolean ploppedGamepiece) {}
