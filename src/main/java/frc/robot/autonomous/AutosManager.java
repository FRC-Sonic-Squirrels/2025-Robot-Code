package frc.robot.autonomous;

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
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.records.AutoDescriptor;
import frc.robot.autonomous.records.AutoDescriptor.StartingLocation;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.autonomous.records.CoralStationLocation;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.autonomous.records.ScoringLocation.ReefSide;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutosManager {
  private static final LoggerGroup logGroupSysid = LoggerGroup.build("Sysid");
  private static final LoggerEntry.EnumValue<SysIdRoutineLog.State> logSwerveSysidState =
      logGroupSysid.buildEnum("swervesysidstate");

  private final AutosSubsystems subsystems;
  private final RobotConfig config;
  private final BooleanSupplier flipAuto;

  public final boolean includeDebugPaths = false;

  public record Auto(String name, Command command, Pose2d initPose) {}

  public AutosManager(
      AutosSubsystems subsystems,
      RobotConfig config,
      LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<Auto>> stringToAutoSupplierMap,
      BooleanSupplier flipAuto) {
    this.subsystems = subsystems;
    this.config = config;
    this.flipAuto = flipAuto;

    fillChooserAndMap(chooser, stringToAutoSupplierMap);
  }

  private List<Supplier<Auto>> allCompetitionAutos() {
    var list = new ArrayList<Supplier<Auto>>();

    list.add(this::doNothing);
    list.add(this::auto_IKLJ);

    if (includeDebugPaths) {
      list.add(this::swerveCharacterization);
      // list.add(() -> testPath("TestDrive1Meter", true));
      // list.add(() -> testPath("TestDrive10Meter", true));
      // list.add(() -> testPath("TestDrive2Meters", true));
      // list.add(() -> testPath("TestDrive2MetersRotating", true));
      // list.add(() -> testPath("TestDrive2MetersThenLeft", true));
      // list.add(() -> testPath("TestDrive2MetersThenLeftRotating", true));
      // list.add(() -> testPath("TestCircle", true));
      // list.add(() -> testPath("TestCircle", false, "TestCircleDontResetPose"));
      // list.add(() -> testPath("TestZigZag", false));
    }

    return list;
  }

  private void fillChooserAndMap(
      LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<Auto>> stringToAutoSupplierMap) {
    var compAutos = allCompetitionAutos();

    for (int i = 0; i < compAutos.size(); i++) {
      var supplier = compAutos.get(i);
      var name = supplier.get().name;
      if (i == 0) {
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

  private Auto auto_IKLJ() {
    List<ScoringLocation> scoringLocations = new ArrayList<>();
    List<CoralStationLocation> coralStationLocations = new ArrayList<>();

    scoringLocations.add(new ScoringLocation(ReefSide.CI, ScoringLevel.L4));
    coralStationLocations.add(CoralStationLocation.IA);

    scoringLocations.add(new ScoringLocation(ReefSide.CK, ScoringLevel.L4));
    coralStationLocations.add(CoralStationLocation.IA);

    scoringLocations.add(new ScoringLocation(ReefSide.CL, ScoringLevel.L4));
    coralStationLocations.add(CoralStationLocation.IA);

    scoringLocations.add(new ScoringLocation(ReefSide.CJ, ScoringLevel.L4));
    coralStationLocations.add(CoralStationLocation.IA);

    var state =
        new AutoStateMachine(
            subsystems,
            new AutoDescriptor(scoringLocations, coralStationLocations, StartingLocation.S1),
            config,
            flipAuto.getAsBoolean());

    return stateToAuto("IKLJ", state);
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
                    drivetrain.getReefPoseEstimatorPose(true),
                    traj,
                    config.getDriveBaseRadius() / 2,
                    config.getAutoTranslationPidController(),
                    config.getAutoTranslationPidController(),
                    config.getAutoThetaPidController());
          }

          @Override
          public void execute() {
            var result =
                helper.calculateChassisSpeeds(
                    drivetrain.getReefPoseEstimatorPose(true), Timer.getFPGATimestamp());
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

  /*
    Naming Convention:

    Starting locations

    S1 ("S" for "Starting", 2nd letter indicates order from non processor side to processor side of starting line)

    Scoring locations

    CA - CL ("C" for "Coral", 2nd letter is based on diagram on page 24 of game manual : https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf)



    Copy these to get waypoints for choreo. If pasted in choreo, they will automatically be turned into waypoints

    CA:
    {"dataType":"choreo/waypoint","x":{"exp":"3.238499 m","val":3.238499},"y":{"exp":"4.191 m","val":4.191},"heading":{"exp":"180 deg","val":3.141592653589793},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CB:
    {"dataType":"choreo/waypoint","x":{"exp":"3.238499 m","val":3.238499},"y":{"exp":"3.8608 m","val":3.8608},"heading":{"exp":"180 deg","val":3.141592653589793},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CC:
    {"dataType":"choreo/waypoint","x":{"exp":"3.720994 m","val":3.720994},"y":{"exp":"3.025095 m","val":3.025095},"heading":{"exp":"-120 deg","val":-2.0943951023931953},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CD:
    {"dataType":"choreo/waypoint","x":{"exp":"4.006955 m","val":4.006955},"y":{"exp":"2.859995 m","val":2.859995},"heading":{"exp":"-120 deg","val":-2.0943951023931953},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CE:
    {"dataType":"choreo/waypoint","x":{"exp":"4.971944 m","val":4.971944},"y":{"exp":"2.859995 m","val":2.859995},"heading":{"exp":"-60 deg","val":-1.0471975511965976},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CF:
    {"dataType":"choreo/waypoint","x":{"exp":"5.257905 m","val":5.257905},"y":{"exp":"3.025095 m","val":3.025095},"heading":{"exp":"-60 deg","val":-1.0471975511965976},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CG:
    {"dataType":"choreo/waypoint","x":{"exp":"5.740399 m","val":5.740399},"y":{"exp":"3.8608 m","val":3.8608},"heading":{"exp":"0 deg","val":0},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CH:
    {"dataType":"choreo/waypoint","x":{"exp":"5.740399 m","val":5.740399},"y":{"exp":"4.191 m","val":4.191},"heading":{"exp":"0 deg","val":0},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CI:
    {"dataType":"choreo/waypoint","x":{"exp":"5.257905 m","val":5.257905},"y":{"exp":"5.026704 m","val":5.026704},"heading":{"exp":"60 deg","val":1.0471975511965976},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CJ:
    {"dataType":"choreo/waypoint","x":{"exp":"4.971944 m","val":4.971944},"y":{"exp":"5.191804 m","val":5.191804},"heading":{"exp":"60 deg","val":1.0471975511965976},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CK:
    {"dataType":"choreo/waypoint","x":{"exp":"4.006955 m","val":4.006955},"y":{"exp":"5.191804 m","val":5.191804},"heading":{"exp":"120 deg","val":2.0943951023931953},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    CL:
    {"dataType":"choreo/waypoint","x":{"exp":"3.720994 m","val":3.720994},"y":{"exp":"5.026704 m","val":5.026704},"heading":{"exp":"120 deg","val":2.0943951023931953},"fixTranslation":true,"fixHeading":true,"intervals":40,"overrideIntervals":false,"split":false}

    IA:
    {"dataType":"choreo/waypoint","x":{"exp":"1.487926 m","val":1.487926},"y":{"exp":"7.341086 m","val":7.341086},"heading":{"exp":"126 deg","val":2.199114857512855},"fixTranslation":true,"fixHeading":true,"intervals":28,"overrideIntervals":false,"split":false}

    IB:


    IC:


    ID:

    S1:
    {"dataType":"choreo/waypoint","x":{"exp":"7.1 m","val":7.1},"y":{"exp":"5.026704 m","val":5.026704},"heading":{"exp":"60 deg","val":1.0471975511965976},"fixTranslation":true,"fixHeading":true,"intervals":19,"overrideIntervals":false,"split":false}


    CA: x 3.238499 y 4.191000 r 180
    CB: x 3.238499 y 3.860800 r 180
    CC: x 3.720994 y 3.025095 r -120
    CD: x 4.006955 y 2.859995 r -120
    CE: x 4.971944 y 2.859995 r -60
    CF: x 5.257905 y 3.025095 r -60
    CG: x 5.740399 y 3.860800 r 0
    CH: x 5.740399 y 4.191000 r 0
    CI: x 5.257905 y 5.026704 r 60
    CJ: x 4.971944 y 5.191804 r 60
    CK: x 4.006955 y 5.191804 r 120
    CL: x 3.720994 y 5.026704 r 120

    IA: x 1.487926 y 7.341086 r 126
    IB: x 0.707063 y 6.773755 r 126
    IC: x 0.707063 y 1.278044 r -126
    ID: x 1.487926 y 0.710713 r -126

    S1: x 7.100000 y 5.026704 r 60
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

  private Auto stateToAuto(String name, AutoStateMachine state) {
    return new Auto(name, state.asCommand(), state.initPose());
  }
}
