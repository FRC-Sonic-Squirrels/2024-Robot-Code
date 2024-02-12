package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
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

  // FIXME: add all other subssystems

  // FIXME: IMPORTANT!!!!!!!!!!! full auto traj files are currently BROKEN due to bug with choreo.
  // issue has been submitted
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
    // list.add(this::testAuto);
    // list.add(this::middle5Piece);
    // list.add(this::middle6Piece);
    // list.add(this::middle8Piece);
    // list.add(this::amp2Piece);
    // list.add(this::amp3Piece);
    // list.add(this::amp5Piece);
    // list.add(this::amp6Piece);
    // list.add(this::source2Piece);
    // list.add(this::source3Piece);
    // list.add(this::source4Piece);
    // list.add(this::source5Piece);
    // list.add(this::source6Piece);

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

    AutoSubstateMachine substateMachine1 =
        new AutoSubstateMachine(
            drivetrain,
            shooter,
            endEffector,
            intake,
            config,
            "sourceAuto.1",
            "G5S3",
            visionGamepiece::getClosestGamepiece);

    AutoSubstateMachine substateMachine2 =
        new AutoSubstateMachine(
            drivetrain,
            shooter,
            endEffector,
            intake,
            config,
            "S3G4",
            "G4S2",
            visionGamepiece::getClosestGamepiece);

    AutoSubstateMachine substateMachine3 =
        new AutoSubstateMachine(
            drivetrain,
            shooter,
            endEffector,
            intake,
            config,
            "S2G3",
            "G3S1",
            visionGamepiece::getClosestGamepiece);

    AutoSubstateMachine substateMachine4 =
        new AutoSubstateMachine(
            drivetrain,
            shooter,
            endEffector,
            intake,
            config,
            "S1G2",
            "G2S1",
            visionGamepiece::getClosestGamepiece);

    AutoSubstateMachine substateMachine5 =
        new AutoSubstateMachine(
            drivetrain,
            shooter,
            endEffector,
            intake,
            config,
            "S1G1",
            "G1S1",
            visionGamepiece::getClosestGamepiece);

    AutoStateMachine state =
        new AutoStateMachine(
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

  // private Auto testAuto() {
  //   return new Auto(
  //       "testAuto",
  //       generateFollowPathCommand("TestAuto", false, new AutoRotationState[] {}),
  //       Choreo.getTrajectory("TestAuto").getInitialPose());
  // }

  // private Auto middle5Piece() {
  //   return new Auto(
  //       "middle5Piece",
  //       generateFollowPathCommand(
  //           "middleAuto.1",
  //           true,
  //           new AutoRotationState[] {
  //             new AutoRotationState(ChoreoRotationMode.ROTATE_TO_GAMEPIECE, 0.0),
  //             new AutoRotationState(ChoreoRotationMode.FOLLOW_PATH, 0.48),
  //             new AutoRotationState(ChoreoRotationMode.ROTATE_TO_GAMEPIECE, 1.05),
  //             new AutoRotationState(ChoreoRotationMode.FOLLOW_PATH, 4.35)
  //           }),
  //       Choreo.getTrajectory("middleAuto.1").getInitialPose());
  // }

  // private Auto middle6Piece() {
  //   return new Auto(
  //       "middle6Piece",
  //       Commands.sequence(
  //           middle5Piece().command,
  //           generateFollowPathCommand("middleAuto.2", true, new AutoRotationState[] {})),
  //       Choreo.getTrajectory("middleAuto.1").getInitialPose());
  // }

  // private Auto middle8Piece() {
  //   return new Auto(
  //       "middle8Piece",
  //       Commands.sequence(
  //           middle6Piece().command,
  //           generateFollowPathCommand("middleAuto.3", true, new AutoRotationState[] {})),
  //       Choreo.getTrajectory("middleAuto").getInitialPose());
  // }

  // private Auto amp2Piece() {
  //   return new Auto(
  //       "amp2Piece",
  //       generateFollowPathCommand("ampAuto.1", true, new AutoRotationState[] {}),
  //       Choreo.getTrajectory("ampAuto.1").getInitialPose());
  // }

  // private Auto amp3Piece() {
  //   return new Auto(
  //       "amp3Piece",
  //       Commands.sequence(
  //           amp2Piece().command,
  //           generateFollowPathCommand("ampAuto.2", true, new AutoRotationState[] {})),
  //       Choreo.getTrajectory("ampAuto.1").getInitialPose());
  // }

  // private Auto amp5Piece() {
  //   return new Auto(
  //       "amp5Piece",
  //       Commands.sequence(
  //           amp3Piece().command,
  //           generateFollowPathCommand("ampAuto.3", true, new AutoRotationState[] {})),
  //       Choreo.getTrajectory("ampAuto.1").getInitialPose());
  // }

  // private Auto amp6Piece() {
  //   return new Auto(
  //       "amp6Piece",
  //       Commands.sequence(
  //           amp5Piece().command,
  //           generateFollowPathCommand("ampAuto.4", true, new AutoRotationState[] {})),
  //       Choreo.getTrajectory("ampAuto.1").getInitialPose());
  // }

  // private Auto source2Piece() {
  //   return new Auto(
  //       "source2Piece",
  //       generateFollowPathCommand("sourceAuto.1", true, new AutoRotationState[] {}),
  //       Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  // }

  // private Auto source3Piece() {
  //   return new Auto(
  //       "source3Piece",
  //       Commands.sequence(
  //           source2Piece().command,
  //           generateFollowPathCommand("sourceAuto.2", true, new AutoRotationState[] {})),
  //       Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  // }

  // private Auto source4Piece() {
  //   return new Auto(
  //       "source4Piece",
  //       Commands.sequence(
  //           source3Piece().command,
  //           generateFollowPathCommand("sourceAuto.3", true, new AutoRotationState[] {})),
  //       Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  // }

  // private Auto source5Piece() {
  //   return new Auto(
  //       "source5Piece",
  //       Commands.sequence(
  //           source4Piece().command,
  //           generateFollowPathCommand("sourceAuto.4", true, new AutoRotationState[] {})),
  //       Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  // }

  // private Auto source6Piece() {
  //   return new Auto(
  //       "source6Piece",
  //       Commands.sequence(
  //           source5Piece().command,
  //           generateFollowPathCommand("sourceAuto.5", true, new AutoRotationState[] {})),
  //       Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  // }

  // private Command generateFollowPathCommand(
  //     String trajName, boolean intaking, AutoRotationState[] rotationStates, AutoEvent... events)
  // {
  //   ChoreoTrajectory traj = Choreo.getTrajectory(trajName);
  //   Command command =
  //       Choreo.choreoSwerveCommand(
  //           traj,
  //           drivetrain::getPoseEstimatorPose,
  //           choreoSwerveController(
  //               config.getAutoTranslationPidController(),
  //               config.getAutoTranslationPidController(),
  //               config.getAutoThetaPidController(),
  //               shooter::getRPM,
  //               rotationStates),
  //           (ChassisSpeeds speeds) -> drivetrain.runVelocity(speeds),
  //           Constants::isRedAlliance,
  //           drivetrain);
  //   for (AutoEvent autoEvent : events) {
  //     command =
  //         command.deadlineWith(
  //             Commands.sequence(Commands.waitSeconds(autoEvent.timeSeconds), autoEvent.command));
  //   }
  //   if (intaking) {
  //     command = command.deadlineWith(new IntakeGamepiece(intake));
  //   }
  //   return command;
  // }

  // // work in progress

  // // private AutoEvent[] setShootPoints(double[] timesSec){

  // //   ArrayList<AutoEvent> autoEvents = new ArrayList<>();

  // //   for (double timeSec : timesSec) {
  // //     autoEvents.add(new AutoEvent(new IndexGamepiece(endEffector), timeSec));
  // //   }

  // //   autoEvents.add(new ShooterShootMode(shooter, drivetrain))

  // //   return autoEvents.toArray(new AutoEvent[]{});
  // // }

  // /**
  //  * Creates a control function for following a ChoreoTrajectoryState.
  //  *
  //  * @param xController A PIDController for field-relative X translation (input: X error in
  // meters,
  //  *     output: m/s).
  //  * @param yController A PIDController for field-relative Y translation (input: Y error in
  // meters,
  //  *     output: m/s).
  //  * @param rotationController A PIDController for robot rotation (input: heading error in
  // radians,
  //  *     output: rad/s). This controller will have its continuous input range set to -pi..pi by
  //  *     ChoreoLib.
  //  * @return A ChoreoControlFunction to track ChoreoTrajectoryStates. This function returns
  //  *     robot-relative ChassisSpeeds.
  //  */
  // private ChoreoControlFunction choreoSwerveController(
  //     PIDController xController,
  //     PIDController yController,
  //     PIDController rotationController,
  //     Supplier<Double> shooterRPM,
  //     AutoRotationState... rotationStates) {
  //   rotationController.enableContinuousInput(-Math.PI, Math.PI);
  //   return (pose, referenceState) -> {
  //     double xFF = referenceState.velocityX;
  //     double yFF = referenceState.velocityY;
  //     double rotationFF;

  //     AutoRotationState lastState = new AutoRotationState(ChoreoRotationMode.FOLLOW_PATH, -1.0);

  //     for (AutoRotationState autoRotationState : rotationStates) {
  //       if (autoRotationState.timestamp <= referenceState.timestamp
  //           && autoRotationState.timestamp >= lastState.timestamp) lastState = autoRotationState;
  //     }

  //     double xFeedback = xController.calculate(pose.getX(), referenceState.x);
  //     double yFeedback = yController.calculate(pose.getY(), referenceState.y);

  //     double xVel = xFF + xFeedback;
  //     double yVel = yFF + yFeedback;

  //     double targetHeading;
  //     switch (lastState.rotationMode) {
  //       case FOLLOW_PATH:
  //         targetHeading = referenceState.heading;
  //         rotationFF = referenceState.angularVelocity;
  //         break;
  //       case ROTATE_TO_GAMEPIECE:
  //         targetHeading = referenceState.heading;
  //         rotationFF = referenceState.angularVelocity;
  //         break;
  //       case ROTATE_TO_SPEAKER:
  //         targetHeading = referenceState.heading;
  //         rotationFF = referenceState.angularVelocity;
  //         break;
  //       default:
  //         targetHeading = referenceState.heading;
  //         rotationFF = referenceState.angularVelocity;
  //         break;
  //     }
  //     double rotationFeedback =
  //         rotationController.calculate(pose.getRotation().getRadians(), targetHeading);

  //     Logger.recordOutput("autonomous/targetHeading", targetHeading);
  //     Logger.recordOutput("autonomous/rotationMode", lastState.rotationMode);

  //     return ChassisSpeeds.fromFieldRelativeSpeeds(
  //         xVel, yVel, rotationFF + rotationFeedback, pose.getRotation());
  //   };
  // }

  // public static enum ChoreoRotationMode {
  //   FOLLOW_PATH,
  //   ROTATE_TO_GAMEPIECE,
  //   ROTATE_TO_SPEAKER
  // }

  /*
   * Poses (xMeters, yMeters, rotRad):
   * Gamepieces (8.273, yMeters, 0.0):
   * G1: yMeters = 7.474
   * G2: yMeters = 5.792
   * G3: yMeters = 4.11
   * G4: yMeters = 2.428
   * G5: yMeters = 0.742
   * Scoring Locations:
   * S1: (5.645289421081543, 6.458367347717285, 0.18100366192345307)
   * S2: (4.60924768447876, 4.741092681884766, -0.1594733550343424)
   * S3: (5.801405429840088, 1.6471593379974365, -0.49934669492999156)
   */
}
