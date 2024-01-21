package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ControlMode;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {

  private SingleJointedArmSim pivot =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          Constants.ShooterConstants.Pitch.GEARING,
          SingleJointedArmSim.estimateMOI(Units.feetToMeters(1.5), 20.0),
          Constants.ShooterConstants.SHOOTER_LENGTH,
          Constants.ShooterConstants.Pitch.MIN_ANGLE_RAD,
          Constants.ShooterConstants.Pitch.MAX_ANGLE_RAD,
          false,
          Constants.ShooterConstants.Pitch.SIM_INITIAL_ANGLE);

  private TalonFX leadMotor = new TalonFX(0);
  private TalonFXSimState leadMotorSim = new TalonFXSimState(leadMotor);
  private TalonFX followMotor = new TalonFX(0);
  private TalonFXSimState followMotorSim = new TalonFXSimState(followMotor);

  private final ProfiledPIDController pivotFeedback =
      new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));

  private final PIDController launcherVelController = new PIDController(0, 0, 0);

  private double pivotTargetVelRadPerSec = 0.0;
  private PIDController pivotVelController = new PIDController(60, 0, 0);
  private ControlMode pivotControlMode = ControlMode.VELOCITY;
  private Rotation2d pivotClosedLoopTargetAngle = new Rotation2d();
  private double pivotControlEffort = 0.0;
  private double pivotOpenLoopVolts = 0.0;
  private double pivotKg = 0.0;

  private ControlMode launcherControlMode = ControlMode.VOLTAGE;
  private double launcherOpenLoopVolts = 0.0;
  private double launcherTargetVelRPM = 0.0;

  private StatusSignal<Double> launcherRotorVelocity;
  private StatusSignal<Double> launcherLeadTempCelsius;
  private StatusSignal<Double> launcherFollowTempCelsius;
  private StatusSignal<Double> launcherVoltage;

  public ShooterIOSim() {
    followMotor.setControl(leadMotor.getAppliedControl());

    launcherRotorVelocity = leadMotor.getRotorVelocity();
    launcherLeadTempCelsius = leadMotor.getDeviceTemp();
    launcherFollowTempCelsius = followMotor.getDeviceTemp();
    launcherVoltage = leadMotor.getMotorVoltage();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        launcherRotorVelocity, launcherLeadTempCelsius, launcherFollowTempCelsius, launcherVoltage);

    pivot.update(0.02);

    inputs.pitch = new Rotation2d(pivot.getAngleRads());
    inputs.launcherVoltage = leadMotorSim.getMotorVoltage();

    double ff = Math.cos(pivot.getAngleRads()) * pivotKg;

    if (pivotControlMode.equals(ControlMode.POSITION)) {

      pivotControlEffort =
          pivotFeedback.calculate(
                  inputs.pitch.getRadians(), pivotClosedLoopTargetAngle.getRadians())
              + ff;

      Logger.recordOutput("Shooter/feedForward", ff);
      Logger.recordOutput("Shooter/error", pivotFeedback.getPositionError());

    } else if (pivotControlMode.equals(ControlMode.VELOCITY)) {

      pivotControlEffort =
          pivotVelController.calculate(pivot.getVelocityRadPerSec(), pivotTargetVelRadPerSec) + ff;

    } else {

      pivotControlEffort = pivotOpenLoopVolts;
    }

    if (launcherControlMode.equals(ControlMode.VELOCITY)) {

      leadMotorSim.setSupplyVoltage(launcherTargetVelRPM / 6000.0);

    } else {

      leadMotorSim.setSupplyVoltage(launcherOpenLoopVolts);
    }

    pivot.setInputVoltage(pivotControlEffort);

    Logger.recordOutput("Shooter/targetVelRadPerSec", pivotTargetVelRadPerSec);
    Logger.recordOutput("Shooter/currentVelRadPerSec", pivot.getVelocityRadPerSec());
    Logger.recordOutput(
        "Shooter/pitchController",
        pivotVelController.calculate(pivot.getVelocityRadPerSec(), pivotTargetVelRadPerSec));
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
    launcherControlMode = ControlMode.VOLTAGE;
  }

  @Override
  public void setLauncherPercentOut(double percent) {
    launcherOpenLoopVolts = percent * 12.0;
    launcherControlMode = ControlMode.VOLTAGE;
  }

  @Override
  public void setLauncherRPM(double rpm) {
    launcherTargetVelRPM = rpm;
    launcherControlMode = ControlMode.VELOCITY;
  }

  @Override
  public void setLauncherClosedLoopConstants(double kP, double kI, double kD) {
    launcherVelController.setP(kP);
    launcherVelController.setI(kI);
    launcherVelController.setD(kD);
  }
}
