package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.team2930.TunableNumberGroup;
import frc.robot.Constants;

public class EndEffectorIOSim implements EndEffectorIO {

  private DCMotorSim motor =
      new DCMotorSim(
          DCMotor.getFalcon500(1),
          Constants.EndEffectorConstants.GEARING,
          Constants.EndEffectorConstants.MOI);

  private static final TunableNumberGroup group = new TunableNumberGroup("sim_EndEffector");

  private double timeMarkForNoteIntakingDone = Double.NaN;
  private double timeMarkForNoteDroppingDone = Double.NaN;

  private double voltage = 0.0;

  public EndEffectorIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {
    motor.update(Constants.kDefaultPeriod);
    motor.setInputVoltage(voltage);

    var now = Timer.getFPGATimestamp();

    inputs.intakeSideTOFDistanceInches = 18;
    inputs.shooterSideTOFDistanceInches = 18;

    // Simulate note movements.
    if (Double.isFinite(timeMarkForNoteIntakingDone)) {
      if (now > timeMarkForNoteIntakingDone) {
        inputs.intakeSideTOFDistanceInches = 0;
      }
      if (now > timeMarkForNoteIntakingDone + 0.1) {
        inputs.shooterSideTOFDistanceInches = 0;
      }

    } else if (Double.isFinite(timeMarkForNoteDroppingDone)) {
      if (now < timeMarkForNoteDroppingDone) {
        inputs.intakeSideTOFDistanceInches = 0;
        inputs.shooterSideTOFDistanceInches = 0;
      } else if (now < timeMarkForNoteDroppingDone + 0.1) {
        inputs.shooterSideTOFDistanceInches = 0;
      } else {
        timeMarkForNoteDroppingDone = Double.NaN; // Shooting done, reset marks.
      }
    }
  }

  @Override
  public void setVoltage(double volts) {
    voltage = volts;
  }

  @Override
  public void markStartOfNoteIntaking() {
    if (Double.isNaN(timeMarkForNoteIntakingDone)) {
      timeMarkForNoteIntakingDone = Timer.getFPGATimestamp() + 0.2;
      timeMarkForNoteDroppingDone = Double.NaN;
    }
  }

  @Override
  public void markStartOfNoteDropping() {
    if (Double.isNaN(timeMarkForNoteDroppingDone)) {
      timeMarkForNoteIntakingDone = Double.NaN;
      timeMarkForNoteDroppingDone = Timer.getFPGATimestamp() + 0.2;
    }
  }
}
