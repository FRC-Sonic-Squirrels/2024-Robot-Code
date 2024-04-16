package frc.robot.autonomous.records;

public record PathDescriptor(
    String intakingTraj, String shootingTraj, boolean useVision, boolean ploppedGamepiece) {}
