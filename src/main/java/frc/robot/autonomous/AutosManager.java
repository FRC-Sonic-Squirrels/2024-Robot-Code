package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team2930.AllianceFlipUtil;
import frc.robot.Constants;
import frc.robot.DrivetrainWrapper;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutosManager {
  private DrivetrainWrapper drivetrain;
  private Shooter shooter;
  private EndEffector endEffector;
  private Intake intake;
  private VisionGamepiece visionGamepiece;

  private RobotConfig config;

  private LoggedDashboardChooser<Supplier<Auto>> chooser;

  public record Auto(String name, Command command, Pose2d initPose) {}

  public AutosManager(
      DrivetrainWrapper drivetrain,
      Shooter shooter,
      Intake intake,
      EndEffector endEffector,
      VisionGamepiece visionGamepiece,
      RobotConfig config,
      LoggedDashboardChooser<Supplier<Auto>> chooser) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.visionGamepiece = visionGamepiece;

    this.config = config;

    this.chooser = chooser;
    fillChooser();
  }

  private List<Supplier<Auto>> allCompetitionAutos() {
    var list = new ArrayList<Supplier<Auto>>();

    list.add(this::doNothing);
    list.add(this::sourceAuto);

    return list;
  }

  private void fillChooser() {
    var compAutos = this.allCompetitionAutos();

    for (int i = 0; i < compAutos.size(); i++) {
      var supplier = compAutos.get(i);
      if (i == 0) {
        // FIXME: maybe we dont want do nothing as our default auto? maybe shoot and mobility as
        // default?
        // Do nothing command must be first in list.
        chooser.addDefaultOption(supplier.get().name, supplier);
      } else {
        chooser.addOption(supplier.get().name, supplier);
      }
    }
  }

  private Auto doNothing() {
    return new Auto("doNothing", new InstantCommand(), new Pose2d());
  }

  private Auto sourceAuto() {
    AutoSubstateMachine substateMachine1 = generateSubstateMachine("sourceAuto.1", "G5S3");

    AutoSubstateMachine substateMachine2 = generateSubstateMachine("S3G4", "G4S2");

    AutoSubstateMachine substateMachine3 = generateSubstateMachine("S2G3", "G3S1");

    AutoSubstateMachine substateMachine4 = generateSubstateMachine("S1G2", "G2S1");

    AutoSubstateMachine substateMachine5 = generateSubstateMachine("S1G1", "G1S1");

    AutoStateMachine state =
        new AutoStateMachine(
            drivetrain,
            shooter,
            new AutoSubstateMachine[] {
              substateMachine1,
              substateMachine2,
              substateMachine3,
              substateMachine4,
              substateMachine5
            });
    return new Auto(
        "sourceAuto",
        state.asCommand(),
        Constants.isRedAlliance()
            ? AllianceFlipUtil.mirrorPose2DOverCenterLine(
                Choreo.getTrajectory("sourceAuto.1").getInitialPose())
            : Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  private AutoSubstateMachine generateSubstateMachine(String trajToGP, String trajToShoot) {
    return new AutoSubstateMachine(
        drivetrain,
        shooter,
        endEffector,
        intake,
        config,
        trajToGP,
        trajToShoot,
        visionGamepiece::getClosestGamepiece);
  }
}
