// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import java.util.ArrayList;
import java.util.List;

public class Shooter extends SubsystemBase {
  private static final String ROOT_TABLE = "Shooter";
  private static final double MAX_VOLTAGE = 12;

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  private static final LoggerEntry logInputs = new LoggerEntry(ROOT_TABLE);
  public static final LoggerGroup logGroup = new LoggerGroup(ROOT_TABLE);
  private static final LoggerEntry logPitchDegrees = logGroup.build("PitchDegrees");
  private static final LoggerEntry logTargetPitchDegrees = logGroup.build("TargetPitchDegrees");
  private static final LoggerEntry logNoteInShooter = logGroup.build("NoteInShooter");
  private static final LoggerEntry logPivotPIDLatency = logGroup.build("PivotPIDLatency");

  public static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  private static final TunableNumberGroup groupPivot = group.subgroup("Pivot");
  private static final LoggedTunableNumber pivotkP = groupPivot.build("kP");
  private static final LoggedTunableNumber pivotkD = groupPivot.build("kD");
  private static final LoggedTunableNumber pivotkG = groupPivot.build("kG");
  private static final LoggedTunableNumber pivotClosedLoopMaxVelocityConstraint =
      groupPivot.build("ClosedLoopMaxVelocityConstraint");
  private static final LoggedTunableNumber pivotClosedLoopMaxAccelerationConstraint =
      groupPivot.build("ClosedLoopMaxAccelerationConstraint");

  private static final TunableNumberGroup groupLauncher = group.subgroup("Launcher");
  private static final LoggedTunableNumber launcherkS = groupLauncher.build("kS");
  private static final LoggedTunableNumber launcherkP = groupLauncher.build("kP");
  private static final LoggedTunableNumber launcherkV = groupLauncher.build("kV");
  private static final LoggedTunableNumber launcherClosedLoopMaxAccelerationConstraint =
      groupLauncher.build("ClosedLoopMaxAccelerationConstraint");

  private static final TunableNumberGroup groupKicker = group.subgroup("Kicker");
  private static final LoggedTunableNumber kickerkP = groupKicker.build("kP");
  private static final LoggedTunableNumber kickerkV = groupKicker.build("kV");
  private static final LoggedTunableNumber kickerClosedLoopMaxAccelerationConstraint =
      groupKicker.build("ClosedLoopMaxAccelerationConstraint");

  private static final LoggedTunableNumber pivotToleranceDegrees =
      group.build("pivotToleranceDegrees", 0.5);
  private static final LoggedTunableNumber launcherToleranceRPM =
      group.build("launcherToleranceRPM", 150); // TODO: tune for better tolerance
  public static final LoggedTunableNumber distanceToTriggerNoteDetection =
      group.build("distanceToTriggerNote", 8.0);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_MAESTRO) {
      pivotkP.initDefault(800.0);
      pivotkD.initDefault(0.0);
      pivotkG.initDefault(0.0);
      pivotClosedLoopMaxVelocityConstraint.initDefault(10.0);
      pivotClosedLoopMaxAccelerationConstraint.initDefault(5.0);

      launcherkS.initDefault(0.2949);
      launcherkP.initDefault(0.2);
      launcherkV.initDefault(0.072);
      launcherClosedLoopMaxAccelerationConstraint.initDefault(300.0);

      kickerkP.initDefault(0.6);
      kickerkV.initDefault(0.18);
      kickerClosedLoopMaxAccelerationConstraint.initDefault(300.0);
    } else if (Constants.RobotMode.isSimBot()) {
      pivotkP.initDefault(15.0);
      pivotkD.initDefault(0.0);
      pivotkG.initDefault(0.0);
      pivotClosedLoopMaxVelocityConstraint.initDefault(10.0);
      pivotClosedLoopMaxAccelerationConstraint.initDefault(10.0);

      launcherkS.initDefault(0.0);
      launcherkP.initDefault(0.5);
      launcherkV.initDefault(0.13);
      launcherClosedLoopMaxAccelerationConstraint.initDefault(10.0);

      kickerkP.initDefault(0.0);
      kickerkV.initDefault(0.0);
      kickerClosedLoopMaxAccelerationConstraint.initDefault(0.0);
    }
  }

  private final Trigger noteInShooter = new Trigger(this::getToFActivated);

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Creates a new flat moving average filter
  // Average will be taken over the last 20 samples
  private final LinearFilter pivotPidLatencyfilter = LinearFilter.movingAverage(20);

  private final List<PIDTargetMeasurement> pivotTargetMeasurements = new ArrayList<>();

  private double pivotPidLatency = 0.0;

  private Rotation2d targetPivotPosition = Constants.zeroRotation2d;
  private double targetLauncherRPM = 0.0;

  /** Creates a new ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;

    io.setLauncherClosedLoopConstants(
        launcherkP.get(),
        launcherkV.get(),
        launcherkS.get(),
        launcherClosedLoopMaxAccelerationConstraint.get());

    io.setPivotClosedLoopConstants(
        pivotkP.get(),
        pivotkD.get(),
        pivotkG.get(),
        pivotClosedLoopMaxVelocityConstraint.get(),
        pivotClosedLoopMaxAccelerationConstraint.get());

    io.setKickerClosedLoopConstants(
        kickerkP.get(), kickerkV.get(), 0.0, kickerClosedLoopMaxAccelerationConstraint.get());
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      io.updateInputs(inputs);

      logInputs.info(inputs);
      logPitchDegrees.info(inputs.pivotPosition.getDegrees());
      logTargetPitchDegrees.info(targetPivotPosition.getDegrees());
      logNoteInShooter.info(noteInShooter.getAsBoolean());

      pivotTargetMeasurements.add(
          new PIDTargetMeasurement(
              Timer.getFPGATimestamp(),
              targetPivotPosition,
              inputs.pivotPosition.getRadians() <= targetPivotPosition.getRadians()));
      for (int index = 0; index < pivotTargetMeasurements.size(); index++) {
        PIDTargetMeasurement measurement = pivotTargetMeasurements.get(index);
        if ((measurement.upDirection
                && inputs.pivotPosition.getRadians() >= measurement.targetRot.getRadians())
            || (!measurement.upDirection
                && inputs.pivotPosition.getRadians() <= measurement.targetRot.getRadians())) {
          pivotPidLatency =
              pivotPidLatencyfilter.calculate(Timer.getFPGATimestamp() - measurement.timestamp);
          pivotTargetMeasurements.remove(index);
        } else if (Timer.getFPGATimestamp() - measurement.timestamp >= 1.0) {
          pivotTargetMeasurements.remove(index);
        }
      }
      logPivotPIDLatency.info(pivotPidLatency);

      var hc = hashCode();
      if (launcherkS.hasChanged(hc)
          || launcherkP.hasChanged(hc)
          || launcherkV.hasChanged(hc)
          || launcherClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        io.setLauncherClosedLoopConstants(
            launcherkP.get(),
            launcherkV.get(),
            launcherkS.get(),
            launcherClosedLoopMaxAccelerationConstraint.get());
      }

      if (pivotkP.hasChanged(hc)
          || pivotkD.hasChanged(hc)
          || pivotkG.hasChanged(hc)
          || pivotClosedLoopMaxVelocityConstraint.hasChanged(hc)
          || pivotClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {

        io.setPivotClosedLoopConstants(
            pivotkP.get(),
            pivotkD.get(),
            pivotkG.get(),
            pivotClosedLoopMaxVelocityConstraint.get(),
            pivotClosedLoopMaxAccelerationConstraint.get());
      }

      if (kickerkP.hasChanged(hc)
          || kickerkV.hasChanged(hc)
          || kickerClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        io.setKickerClosedLoopConstants(
            kickerkP.get(), kickerkV.get(), 0.0, kickerClosedLoopMaxAccelerationConstraint.get());
      }
    }
  }

  public Rotation2d getPitch() {
    return inputs.pivotPosition;
  }

  public void setPercentOut(double percent) {
    io.setLauncherVoltage(percent * MAX_VOLTAGE);
  }

  public void setPivotClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {
    io.setPivotClosedLoopConstants(kP, kD, kG, maxProfiledVelocity, maxProfiledAcceleration);
  }

  public void setPivotVoltage(double volts) {
    io.setPivotVoltage(volts);
  }

  public void setPivotPosition(Rotation2d rot) {
    io.setPivotPosition(rot);
    targetPivotPosition = rot;
  }

  public void setLauncherVoltage(double volts) {
    io.setLauncherVoltage(volts);
  }

  public void setLauncherRPM(double rpm) {
    io.setLauncherRPM(rpm);
    targetLauncherRPM = rpm;
  }

  public double getRPM() {
    return inputs.launcherRPM[0];
  }

  public boolean isAtTargetRPM() {
    return isAtTargetRPM(targetLauncherRPM);
  }

  public boolean isAtTargetRPM(double rpm) {
    return Math.abs(inputs.launcherRPM[0] - rpm) <= launcherToleranceRPM.get();
  }

  public double getPivotPIDLatency() {
    return pivotPidLatency;
  }

  public boolean isPivotIsAtTarget() {
    return isPivotIsAtTarget(targetPivotPosition);
  }

  public boolean isPivotIsAtTarget(Rotation2d target) {
    return Math.abs(inputs.pivotPosition.getRadians() - target.getRadians())
        <= Math.toRadians(pivotToleranceDegrees.get());
  }

  public void markStartOfNoteLoading() {
    io.markStartOfNoteLoading();
  }

  public void markStartOfNoteShooting() {
    io.markStartOfNoteShooting();
  }

  public void setKickerPercentOut(double percent) {
    io.setKickerVoltage(percent * Constants.MAX_VOLTAGE);
  }

  public double getKickerPercentOut() {
    return inputs.kickerAppliedVolts / Constants.MAX_VOLTAGE;
  }

  public double getToFDistance() {
    return inputs.timeOfFlightDistance;
  }

  public boolean getToFActivated() {
    return inputs.timeOfFlightDistance <= distanceToTriggerNoteDetection.get();
  }

  public boolean noteInShooter() {
    return noteInShooter.getAsBoolean();
  }

  public void pivotResetHomePosition() {
    io.resetPivotSensorPosition(Constants.ShooterConstants.Pivot.HOME_POSITION);
  }

  public double getPivotVoltage() {
    return inputs.pivotAppliedVolts;
  }

  public double getPivotVelocity() {
    return inputs.pivotVelocityRadsPerSec;
  }

  public void setNeutralMode(NeutralModeValue value) {
    io.setNeutralMode(value);
  }

  public void setKickerVelocity(double revPerMin) {
    io.setKickerVelocity(revPerMin);
  }
}
