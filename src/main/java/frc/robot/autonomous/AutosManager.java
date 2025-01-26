package frc.robot.autonomous;

import choreo.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.autonomous.records.AutoDescriptor;
import frc.robot.autonomous.records.CoralStationLocation;
import frc.robot.autonomous.records.CoralStationLocation.CoralStation;
import frc.robot.autonomous.records.CoralStationLocation.CoralStationSide;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.commands.ScoreCoral.ScoringDirection;
import frc.robot.commands.ScoreCoral.ScoringSide;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutosManager {
  private static final LoggerGroup logGroupSysid = LoggerGroup.build("Sysid");
  private static final LoggerEntry.EnumValue<SysIdRoutineLog.State> logSwerveSysidState =
      logGroupSysid.buildEnum("swervesysidstate");

  private final AutosSubsystems subsystems;
  private final RobotConfig config;

  public final boolean includeDebugPaths = false;

  public record Auto(String name, Command command, Pose2d initPose) {}

  public AutosManager(
      AutosSubsystems subsystems,
      RobotConfig config,
      LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<Auto>> stringToAutoSupplierMap) {
    this.subsystems = subsystems;
    this.config = config;

    fillChooserAndMap(chooser, stringToAutoSupplierMap);
  }

  private List<Supplier<Auto>> allCompetitionAutos() {
    var list = new ArrayList<Supplier<Auto>>();

    list.add(this::doNothing);

    if (includeDebugPaths) {
      list.add(this::auto_IKLA);
      list.add(this::swerveCharacterization);
      list.add(() -> testPath("TestDrive1Meter", true));
      list.add(() -> testPath("TestDrive10Meter", true));
      list.add(() -> testPath("TestDrive2Meters", true));
      list.add(() -> testPath("TestDrive2MetersRotating", true));
      list.add(() -> testPath("TestDrive2MetersThenLeft", true));
      list.add(() -> testPath("TestDrive2MetersThenLeftRotating", true));
      list.add(() -> testPath("TestCircle", true));
      list.add(() -> testPath("TestCircle", false, "TestCircleDontResetPose"));
      list.add(() -> testPath("TestZigZag", false));
      // list.add(this::characterization);
    }

    return list;
  }

  private void fillChooserAndMap(
      LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<Auto>> stringToAutoSupplierMap) {
    var compAutos = this.allCompetitionAutos();

    for (int i = 0; i < compAutos.size(); i++) {
      var supplier = compAutos.get(i);
      var name = supplier.get().name;
      if (i == 0) {
        // FIXME: maybe we dont want do nothing as our default auto? maybe shoot and mobility as
        // default?
        // Do nothing command must be first in list.
        chooser.addDefaultOption(name, name);
      } else {
        chooser.addOption(name, name);
      }

      stringToAutoSupplierMap.put(name, supplier);
    }
  }

  public Auto doNothing() {
    return new Auto("doNothing", new InstantCommand(), Constants.zeroPose2d);
  }

  private Auto auto_IKLA() {
    List<ScoringLocation> scoringLocations = new ArrayList<>();
    List<CoralStationLocation> coralStationLocations = new ArrayList<>();

    scoringLocations.add(
        new ScoringLocation(ScoringSide.FAR_LEFT, ScoringDirection.RIGHT, ScoringLevel.L4));
    coralStationLocations.add(new CoralStationLocation(CoralStation.LEFT, CoralStationSide.LEFT));

    scoringLocations.add(
        new ScoringLocation(ScoringSide.NEAR_LEFT, ScoringDirection.LEFT, ScoringLevel.L4));
    coralStationLocations.add(new CoralStationLocation(CoralStation.LEFT, CoralStationSide.LEFT));

    scoringLocations.add(
        new ScoringLocation(ScoringSide.NEAR_LEFT, ScoringDirection.RIGHT, ScoringLevel.L4));
    coralStationLocations.add(new CoralStationLocation(CoralStation.LEFT, CoralStationSide.RIGHT));

    scoringLocations.add(
        new ScoringLocation(ScoringSide.NEAR_MID, ScoringDirection.LEFT, ScoringLevel.L4));
    coralStationLocations.add(new CoralStationLocation(CoralStation.LEFT, CoralStationSide.RIGHT));

    var state =
        new AutoStateMachine(
            subsystems, new AutoDescriptor(scoringLocations, coralStationLocations));

    return new Auto(
        "IKLA",
        state.asCommand(),
        Choreo.loadTrajectory("S1_I") // TODO: make this a call to the state
            .orElseThrow()
            .getInitialPose(Constants.isRedAlliance())
            .get());
  }

  private Auto testPath(String pathName, boolean useInitialPose) {
    return testPath(pathName, useInitialPose, pathName);
  }

  private Auto testPath(String pathName, boolean useInitialPose, String autoName) {
    var traj = ChoreoTrajectoryWithName.getTrajectory(pathName);
    return new Auto(
        autoName,
        new Command() {
          private final DrivetrainWrapper drivetrain = subsystems.drivetrain();
          ChoreoHelper helper;
          boolean atEndOfPath;

          @Override
          public void initialize() {
            drivetrain.setPose(Constants.zeroPose2d);
            helper =
                new ChoreoHelper(
                    Timer.getFPGATimestamp(),
                    drivetrain.getPoseEstimatorPose(true),
                    traj,
                    config.getDriveBaseRadius() / 2,
                    1.0,
                    config.getAutoTranslationPidController(),
                    config.getAutoTranslationPidController(),
                    config.getAutoThetaPidController());
          }

          @Override
          public void execute() {
            var result =
                helper.calculateChassisSpeeds(
                    drivetrain.getPoseEstimatorPose(true), Timer.getFPGATimestamp());
            drivetrain.setVelocityOverride(result.chassisSpeeds());
            atEndOfPath = result.atEndOfPath();
          }

          @Override
          public void end(boolean interrupted) {
            drivetrain.resetVelocityOverride();
          }

          @Override
          public boolean isFinished() {
            return atEndOfPath;
          }
        },
        useInitialPose ? traj.getInitialPose(true) : null);
  }

  // private Auto characterization() {
  //   PathPlannerPath path = PathPlannerPath.fromPathFile("Characterization");
  //   return new Auto(
  //       "Characterization",
  //       Commands.runOnce(() -> subsystems.drivetrain().setPose(Constants.zeroPose2d))
  //           .andThen(AutoBuilder.followPath(path))
  //           .finallyDo(subsystems.drivetrain()::resetVelocityOverride),
  //       Constants.zeroPose2d);
  // }

  /* Copy these to get waypoints for choreo. If pasted in choreo, they will automatically be turned into waypoints

    A:
    {"dataType":"choreo/waypoint","x":{"exp":"3.238499 m","val":3.238499},"y":{"exp":"4.191 m","val":4.191},"heading":{"exp":"180 deg","val":3.141592653589793},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    B:
    {"dataType":"choreo/waypoint","x":{"exp":"3.238499 m","val":3.238499},"y":{"exp":"3.8608 m","val":3.8608},"heading":{"exp":"180 deg","val":3.141592653589793},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    C:
    {"dataType":"choreo/waypoint","x":{"exp":"3.720994 m","val":3.720994},"y":{"exp":"3.025095 m","val":3.025095},"heading":{"exp":"-120 deg","val":-2.0943951023931953},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    D:
    {"dataType":"choreo/waypoint","x":{"exp":"4.006955 m","val":4.006955},"y":{"exp":"2.859995 m","val":2.859995},"heading":{"exp":"-120 deg","val":-2.0943951023931953},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    E:
    {"dataType":"choreo/waypoint","x":{"exp":"4.971944 m","val":4.971944},"y":{"exp":"2.859995 m","val":2.859995},"heading":{"exp":"-60 deg","val":-1.0471975511965976},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    F:
    {"dataType":"choreo/waypoint","x":{"exp":"5.257905 m","val":5.257905},"y":{"exp":"3.025095 m","val":3.025095},"heading":{"exp":"-60 deg","val":-1.0471975511965976},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    G:
    {"dataType":"choreo/waypoint","x":{"exp":"5.740399 m","val":5.740399},"y":{"exp":"3.8608 m","val":3.8608},"heading":{"exp":"0 deg","val":0},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    H:
    {"dataType":"choreo/waypoint","x":{"exp":"5.740399 m","val":5.740399},"y":{"exp":"4.191 m","val":4.191},"heading":{"exp":"0 deg","val":0},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    I:
    {"dataType":"choreo/waypoint","x":{"exp":"5.257905 m","val":5.257905},"y":{"exp":"5.026704 m","val":5.026704},"heading":{"exp":"60 deg","val":1.0471975511965976},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    J:
    {"dataType":"choreo/waypoint","x":{"exp":"4.971944 m","val":4.971944},"y":{"exp":"5.191804 m","val":5.191804},"heading":{"exp":"60 deg","val":1.0471975511965976},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    K:
    {"dataType":"choreo/waypoint","x":{"exp":"4.006955 m","val":4.006955},"y":{"exp":"5.191804 m","val":5.191804},"heading":{"exp":"120 deg","val":2.0943951023931953},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    L:
    {"dataType":"choreo/waypoint","x":{"exp":"3.720994 m","val":3.720994},"y":{"exp":"5.026704 m","val":5.026704},"heading":{"exp":"120 deg","val":2.0943951023931953},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    A: x 3.238499 y 4.191000 r 180
    B: x 3.238499 y 3.860800 r 180
    C: x 3.720994 y 3.025095 r -120
    D: x 4.006955 y 2.859995 r -120
    E: x 4.971944 y 2.859995 r -60
    F: x 5.257905 y 3.025095 r -60
    G: x 5.740399 y 3.860800 r 0
    H: x 5.740399 y 4.191000 r 0
    I: x 5.257905 y 5.026704 r 60
    J: x 4.971944 y 5.191804 r 60
    K: x 4.006955 y 5.191804 r 120
    L: x 3.720994 y 5.026704 r 120
  */

  public Auto swerveCharacterization() {
    var sysidConfig =
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> {
              if (state != SysIdRoutineLog.State.kNone) {
                logSwerveSysidState.info(state);
              }
            });

    var mechanism = new SysIdRoutine(sysidConfig, subsystems.drivetrain().getSysIdMechanism());

    var command1 = mechanism.quasistatic(Direction.kForward);
    var command2 = mechanism.quasistatic(Direction.kReverse);
    var command3 = mechanism.dynamic(Direction.kForward);
    var command4 = mechanism.dynamic(Direction.kReverse);

    var finalCommand =
        command1
            // .andThen(Commands.waitSeconds(1.0))
            .andThen(command2)
            // .andThen(Commands.waitSeconds(1.0))
            .andThen(command3)
            // .andThen(Commands.waitSeconds(1.0))
            .andThen(command4)
            .andThen(Commands.runOnce(() -> logSwerveSysidState.info(SysIdRoutineLog.State.kNone)));

    return new Auto("swerveCharacterization", finalCommand, Constants.zeroPose2d);
  }
}
