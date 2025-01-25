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
import frc.robot.autonomous.records.PathDescriptor;
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
      list.add(this::portableAuto);
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

  private Auto portableAuto() {
    List<PathDescriptor> paths = new ArrayList<>();
    paths.add(new PathDescriptor(null, null));
    paths.add(new PathDescriptor(null, null));
    var state = new AutoStateMachine(subsystems, paths);
    // TODO: add throw message?
    return new Auto(
        "TestPortable",
        state.asCommand(),
        Choreo.loadTrajectory("TestPortable1")
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
