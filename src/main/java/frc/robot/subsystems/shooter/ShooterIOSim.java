package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ControlMode;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {

  private SingleJointedArmSim pivot =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          Constants.ShooterConstants.Pivot.GEARING,
          SingleJointedArmSim.estimateMOI(Units.feetToMeters(1.5), 20.0),
          Constants.ShooterConstants.SHOOTER_LENGTH,
          Constants.ShooterConstants.Pivot.MIN_ANGLE_RAD,
          Constants.ShooterConstants.Pivot.MAX_ANGLE_RAD,
          false,
          Constants.ShooterConstants.Pivot.SIM_INITIAL_ANGLE);

  private DCMotorSim launcherMotorSim =
      new DCMotorSim(
          DCMotor.getFalcon500Foc(2),
          Constants.ShooterConstants.Launcher.GEARING,
          Constants.ShooterConstants.Launcher.MOI);

  private DCMotorSim kickerMotorSim =
      new DCMotorSim(
          DCMotor.getFalcon500Foc(1),
          Constants.ShooterConstants.Kicker.GEARING,
          Constants.ShooterConstants.Kicker.MOI);

  private final ProfiledPIDController pivotFeedback =
      new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));

  private double pivotTargetVelRadPerSec = 0.0;
  private PIDController pivotVelController = new PIDController(60, 0, 0);
  private ControlMode pivotControlMode = ControlMode.VELOCITY;
  private Rotation2d pivotClosedLoopTargetAngle = new Rotation2d();
  private double pivotControlEffort = 0.0;
  private double pivotOpenLoopVolts = 0.0;
  private double pivotKg = 0.0;

  private double launcherOpenLoopVolts = 0.0;

  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    pivot.update(0.02);
    launcherMotorSim.update(0.02);
    kickerMotorSim.update(0.02);

    pivotFeedback.setTolerance(1.0);

    inputs.pitch = new Rotation2d(pivot.getAngleRads());
    inputs.RPM = launcherMotorSim.getAngularVelocityRPM();

    // double ff = Math.cos(pivot.getAngleRads()) * pivotKg;
    double ff = 0.0;

    if (pivotControlMode.equals(ControlMode.POSITION)) {

      pivotControlEffort =
          pivotFeedback.calculate(
                  inputs.pitch.getRadians(), pivotClosedLoopTargetAngle.getRadians())
              + ff;

      Logger.recordOutput("Shooter/error", pivotFeedback.getPositionError());

    } else if (pivotControlMode.equals(ControlMode.VELOCITY)) {

      pivotControlEffort =
          pivotVelController.calculate(pivot.getVelocityRadPerSec(), pivotTargetVelRadPerSec) + ff;

    } else {

      pivotControlEffort = pivotOpenLoopVolts;
    }

    Logger.recordOutput("Shooter/feedForward", ff);

    launcherMotorSim.setInputVoltage(launcherOpenLoopVolts);

    pivot.setInputVoltage(MathUtil.clamp(pivotControlEffort, -12.0, 12.0));
    Logger.recordOutput("Shooter/outputVoltage", MathUtil.clamp(pivotControlEffort, -12.0, 12.0));

    Logger.recordOutput("Shooter/targetVelRadPerSec", pivotTargetVelRadPerSec);
    Logger.recordOutput("Shooter/currentVelRadPerSec", pivot.getVelocityRadPerSec());
    Logger.recordOutput(
        "Shooter/pitchController",
        pivotVelController.calculate(pivot.getVelocityRadPerSec(), pivotTargetVelRadPerSec));
    Logger.recordOutput("Shooter/pivotControlMode", pivotControlMode.toString());
    Logger.recordOutput("Shooter/targetPositionDegrees", pivotClosedLoopTargetAngle.getDegrees());
  }

  @Override
  public void setPivotVel(double radPerSecond) {
    pivotTargetVelRadPerSec = radPerSecond;
    pivotControlMode = ControlMode.VELOCITY;
  }

  @Override
  public void setPivotPosition(Rotation2d rot) {
    pivotClosedLoopTargetAngle = rot;
    pivotControlMode = ControlMode.POSITION;
  }

  @Override
  public void setPivotClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {

    this.pivotKg = kG;
    pivotFeedback.setP(kP);
    pivotFeedback.setD(kD);
    pivotFeedback.setConstraints(new Constraints(maxProfiledVelocity, maxProfiledAcceleration));
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotOpenLoopVolts = volts;
    pivotControlMode = ControlMode.VOLTAGE;
  }

  @Override
  public void setLauncherVoltage(double volts) {
    launcherOpenLoopVolts = volts;
  }

  @Override
  public void setLauncherPercentOut(double percent) {
    launcherOpenLoopVolts = percent * 12.0;
  }
}
