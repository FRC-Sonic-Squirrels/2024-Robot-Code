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

  private static final LoggerGroup logInputs = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_pivotPosition =
      logInputs.buildDecimal("PivotPosition");
  private static final LoggerEntry.Decimal logInputs_pivotVelocityRadsPerSec =
      logInputs.buildDecimal("PivotVelocityRadsPerSec");
  private static final LoggerEntry.Decimal logInputs_pivotAppliedVolts =
      logInputs.buildDecimal("PivotAppliedVolts");
  private static final LoggerEntry.Decimal logInputs_pivotCurrentAmps =
      logInputs.buildDecimal("PivotCurrentAmps");
  private static final LoggerEntry.DecimalArray logInputs_launcherRPM =
      logInputs.buildDecimalArray("LauncherRPM");
  private static final LoggerEntry.DecimalArray logInputs_launcherAppliedVolts =
      logInputs.buildDecimalArray("LauncherAppliedVolts");
  private static final LoggerEntry.DecimalArray logInputs_launcherCurrentAmps =
      logInputs.buildDecimalArray("LauncherCurrentAmps");
  private static final LoggerEntry.Decimal logInputs_kickerAppliedVolts =
      logInputs.buildDecimal("KickerAppliedVolts");
  private static final LoggerEntry.Decimal logInputs_kickerCurrentAmps =
      logInputs.buildDecimal("KickerCurrentAmps");
  private static final LoggerEntry.Decimal logInputs_kickerVelocityRPM =
      logInputs.buildDecimal("KickerVelocityRPM");
  private static final LoggerEntry.DecimalArray logInputs_tempsCelcius =
      logInputs.buildDecimalArray("TempsCelcius");
  private static final LoggerEntry.Decimal logInputs_timeOfFlightDistance =
      logInputs.buildDecimal("TimeOfFlightDistance");

  public static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal logPitchDegrees = logGroup.buildDecimal("PitchDegrees");
  private static final LoggerEntry.Decimal logTargetPitchDegrees =
      logGroup.buildDecimal("TargetPitchDegrees");
  private static final LoggerEntry.Bool logNoteInShooter = logGroup.buildBoolean("NoteInShooter");
  private static final LoggerEntry.Decimal logPivotPIDLatency =
      logGroup.buildDecimal("PivotPIDLatency");
  private static final LoggerEntry.Decimal logLauncherRollersDifference =
      logGroup.buildDecimal("LauncherRollersDifference");

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
      group.build("distanceToTriggerNote", 10.0);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_MAESTRO) {
      pivotkP.initDefault(800.0);
      pivotkD.initDefault(0.0);
      pivotkG.initDefault(0.0);
      pivotClosedLoopMaxVelocityConstraint.initDefault(10.0);
      pivotClosedLoopMaxAccelerationConstraint.initDefault(5.0);

      launcherkS.initDefault(0.24);
      launcherkP.initDefault(0.4);
      launcherkV.initDefault(0.072);
      launcherClosedLoopMaxAccelerationConstraint.initDefault(1000.0);

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
  private final ShooterIO.Inputs inputs = new ShooterIO.Inputs();

  // Creates a new flat moving average filter
  // Average will be taken over the last 20 samples
  private final LinearFilter pivotPidLatencyfilter = LinearFilter.movingAverage(20);

  private final List<PIDTargetMeasurement> pivotTargetMeasurements = new ArrayList<>();

  private double pivotPidLatency = 0.0;

  private Rotation2d targetPivotPosition = Constants.zeroRotation2d;
  private double topRollerTargetLauncherRPM = 0.0;
  private double bottomRollerTargetLauncherRPM = 0.0;

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
      logInputs_pivotPosition.info(inputs.pivotPosition);
      logInputs_pivotVelocityRadsPerSec.info(inputs.pivotVelocityRadsPerSec);
      logInputs_pivotAppliedVolts.info(inputs.pivotAppliedVolts);
      logInputs_pivotCurrentAmps.info(inputs.pivotCurrentAmps);
      logInputs_launcherRPM.info(inputs.launcherRPM);
      logInputs_launcherAppliedVolts.info(inputs.launcherAppliedVolts);
      logInputs_launcherCurrentAmps.info(inputs.launcherCurrentAmps);
      logInputs_kickerAppliedVolts.info(inputs.kickerAppliedVolts);
      logInputs_kickerCurrentAmps.info(inputs.kickerCurrentAmps);
      logInputs_kickerVelocityRPM.info(inputs.kickerVelocityRPM);
      logInputs_tempsCelcius.info(inputs.tempsCelcius);
      logInputs_timeOfFlightDistance.info(inputs.timeOfFlightDistance);

      logPitchDegrees.info(inputs.pivotPosition.getDegrees());
      logTargetPitchDegrees.info(targetPivotPosition.getDegrees());
      logNoteInShooter.info(noteInShooter.getAsBoolean());
      logLauncherRollersDifference.info(inputs.launcherRPM[1] - inputs.launcherRPM[0]);

      if (Constants.unusedCode) {
        var timestamp = Timer.getFPGATimestamp();
        var pivotPositionRadians = inputs.pivotPosition.getRadians();

        pivotTargetMeasurements.add(
            new PIDTargetMeasurement(
                timestamp,
                targetPivotPosition,
                pivotPositionRadians <= targetPivotPosition.getRadians()));

        for (var it = pivotTargetMeasurements.iterator(); it.hasNext(); ) {
          var measurement = it.next();
          var targetRot = measurement.targetRot.getRadians();

          double delta = timestamp - measurement.timestamp;
          if ((measurement.upDirection && pivotPositionRadians >= targetRot)
              || (!measurement.upDirection && pivotPositionRadians <= targetRot)) {
            pivotPidLatency = pivotPidLatencyfilter.calculate(delta);
            it.remove();
          } else if (delta >= 1.0) {
            it.remove();
          }
        }
        logPivotPIDLatency.info(pivotPidLatency);
      }

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

  public void setLauncherRPM(double topRollerRPM, double bottomRollerRPM) {
    io.setLauncherRPM(topRollerRPM, bottomRollerRPM);
    topRollerTargetLauncherRPM = topRollerRPM;
    bottomRollerTargetLauncherRPM = bottomRollerRPM;
  }

  public void setLauncherRPM(double rpm) {
    io.setLauncherRPM(rpm, rpm);
    topRollerTargetLauncherRPM = rpm;
    bottomRollerTargetLauncherRPM = rpm;
  }

  public double getRPM() {
    return inputs.launcherRPM[0];
  }

  public boolean isAtTargetRPM() {
    return isAtTargetRPM(topRollerTargetLauncherRPM);
  }

  public boolean isAtTargetRPM(double rpm) {
    return inputs.launcherRPM[0] > rpm - launcherToleranceRPM.get();
  }

  public double getPivotPIDLatency() {
    return pivotPidLatency;
  }

  public boolean isPivotIsAtTarget() {
    return isPivotIsAtTarget(targetPivotPosition);
  }

  public boolean isPivotIsAtTarget(Rotation2d target) {
    return isPivotIsAtTarget(target, Rotation2d.fromDegrees(pivotToleranceDegrees.get()));
  }

  public boolean isPivotIsAtTarget(Rotation2d target, Rotation2d tolerance) {
    return Math.abs(inputs.pivotPosition.getRadians() - target.getRadians())
        <= tolerance.getRadians();
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

  public boolean setNeutralMode(NeutralModeValue value) {
    return io.setNeutralMode(value);
  }

  public void setKickerVelocity(double revPerMin) {
    io.setKickerVelocity(revPerMin);
  }
}
