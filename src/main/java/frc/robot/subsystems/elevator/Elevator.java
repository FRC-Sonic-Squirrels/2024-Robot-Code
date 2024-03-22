// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;

public class Elevator extends SubsystemBase {
  public static final String ROOT_TABLE = "Elevator";

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  private static final LoggerEntry logInputs = new LoggerEntry(ROOT_TABLE);
  private static final LoggerGroup logGroup = new LoggerGroup(ROOT_TABLE);
  private static final LoggerEntry logTargetHeight = logGroup.build("targetHeight");
  public static final LoggerEntry logSIM_ActualTargetHeight =
      logGroup.build("SIM_actualTargetHeight");
  public static final LoggerEntry logSIM_Error = logGroup.build("SIM_error");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kD = group.build("kD");
  private static final LoggedTunableNumber kG = group.build("kG");

  private static final LoggedTunableNumber closedLoopMaxVelocityConstraint =
      group.build("defaultClosedLoopMaxVelocityConstraint");
  private static final LoggedTunableNumber closedLoopMaxAccelerationConstraint =
      group.build("defaultClosedLoopMaxAccelerationConstraint");

  private final LoggedTunableNumber tolerance = group.build("toleranceInches", 0.1);

  private static final Constraints motionMagicDefaultConstraints;

  private Constraints currentMotionMagicConstraints = new Constraints(0.0, 0.0);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {
      kP.initDefault(1.0);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      closedLoopMaxVelocityConstraint.initDefault(640.0);
      closedLoopMaxAccelerationConstraint.initDefault(640.0);
      motionMagicDefaultConstraints = new Constraints(640.0, 640.0);

    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_MAESTRO) {
      kP.initDefault(12.0);
      kD.initDefault(0.0);
      kG.initDefault(0.0);

      closedLoopMaxVelocityConstraint.initDefault(1000.0);
      closedLoopMaxAccelerationConstraint.initDefault(2000.0);

      motionMagicDefaultConstraints = new Constraints(1000.0, 2000.0);
    } else {
      motionMagicDefaultConstraints = new Constraints(640.0, 640.0);
    }
  }

  private double lastServoActivationTime = 0.0;
  private boolean rightServoActive = false;
  private Rotation2d targetServoAngle = Constants.zeroRotation2d;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private Measure<Distance> targetHeight = Units.Meters.zero();

  /** Creates a new ElevatorSubsystem. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    setClosedLoopConstraints(
        kP.get(),
        kD.get(),
        kG.get(),
        new Constraints(
            closedLoopMaxVelocityConstraint.get(), closedLoopMaxAccelerationConstraint.get()));
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      io.updateInputs(inputs);
      logInputs.info(inputs);

      logTargetHeight.info(targetHeight.in(Units.Inches));

      // ---- UPDATE TUNABLE NUMBERS
      var hc = hashCode();
      if (kP.hasChanged(hc)
          || kD.hasChanged(hc)
          || kG.hasChanged(hc)
          || closedLoopMaxVelocityConstraint.hasChanged(hc)
          || closedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        setClosedLoopConstraints(
            kP.get(),
            kD.get(),
            kG.get(),
            new Constraints(
                closedLoopMaxVelocityConstraint.get(), closedLoopMaxAccelerationConstraint.get()));
      }
    }
  }

  public void resetSubsystem() {
    io.setVoltage(0);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setHeight(Measure<Distance> height) {
    io.setHeight(height);
    targetHeight = height;
  }

  public boolean isAtTarget() {
    return Math.abs(targetHeight.in(Units.Inches) - inputs.heightInches) <= tolerance.get();
  }

  public boolean isAtTarget(Measure<Distance> height) {
    return Math.abs(height.in(Units.Inches) - inputs.heightInches) <= tolerance.get();
  }

  public double getHeightInches() {
    return inputs.heightInches;
  }

  public void resetSensorToHomePosition() {
    io.setSensorPosition(Constants.ElevatorConstants.HOME_POSITION);
  }

  public double getVoltage() {
    return inputs.appliedVolts;
  }

  public double getVelocity() {
    return inputs.velocityInchesPerSecond;
  }

  public void setNeutralMode(NeutralModeValue value) {
    io.setNeutralMode(value);
  }

  private void setClosedLoopConstraints(double kp, double kd, double kg, Constraints constraints) {
    currentMotionMagicConstraints =
        new Constraints(constraints.maxVelocity, constraints.maxAcceleration);

    io.setPIDConstraints(kp, kd, kg, constraints);
  }

  public void setMotionMagicConstraints(Constraints constraints) {
    setClosedLoopConstraints(kP.get(), kD.get(), kG.get(), constraints);
  }

  public Constraints getDefaultMotionMagicConstraints() {
    return motionMagicDefaultConstraints;
  }

  public Constraints getCurrentMotionMagicConstraints() {
    return currentMotionMagicConstraints;
  }

  // REACTION ARM
  public void deployReactionArms() {
    io.setReactionArmPosition(
        Constants.ElevatorConstants.ReactionArmConstants.REACTION_ARM_DEPLOY_ROTATIONS);
  }

  public void reactionArmsAmp() {
    io.setReactionArmPosition(
        Constants.ElevatorConstants.ReactionArmConstants.REACTION_ARM_AMP_ROTATIONS);
  }

  public void retractReactionArms() {
    io.setReactionArmPosition(
        Constants.ElevatorConstants.ReactionArmConstants.REACTION_ARM_HOME_ROTATIONS);
  }

  public void resetReactionArmPositions() {
    io.resetReactionArmPosition();
  }

  public void setReactionArmIdleMode(IdleMode idleMode) {
    io.setReactionArmIdleMode(idleMode);
  }
}
