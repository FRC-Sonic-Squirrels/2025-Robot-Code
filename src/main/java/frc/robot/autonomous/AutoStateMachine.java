// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.RobotStates;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.records.AutoDescriptor;
import frc.robot.autonomous.records.AutoDescriptor.StartingLocation;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.autonomous.records.CoralStationLocation;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.commands.ScoreCoral;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.ArrayList;
import java.util.List;

public class AutoStateMachine extends StateMachine {

  private final DrivetrainWrapper wrapper;
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endEffector;
  private final LED led;

  private final List<ChoreoTrajectoryWithName> scoringPaths = new ArrayList<>();
  private final List<ScoringLocation> scoringLocations;
  private final List<ChoreoTrajectoryWithName> coralStationPaths = new ArrayList<>();

  private final RobotConfig config;

  private int scoringIndex = 0;
  private int intakingIndex = 0;

  private ChoreoHelper choreoHelper;

  private Pose2d scoringEndPose = Pose2d.kZero;

  private TunableNumberGroup group = new TunableNumberGroup("AutoStateMachine");
  private LoggedTunableNumber distBeforeScoringMeters = group.build("DistBeforeScoringMeters", 1);

  /** Creates a new AutoSubstateMachine. */
  public AutoStateMachine(
      AutosSubsystems subsystems, AutoDescriptor descriptor, RobotConfig config, boolean flipAuto) {
    super("Auto");

    wrapper = subsystems.drivetrain();
    elevator = subsystems.elevator();
    arm = subsystems.arm();
    endEffector = subsystems.endEffector();
    led = subsystems.led();
    scoringLocations =
        flipAuto ? descriptor.flippedScoringLocations() : descriptor.scoringLocations();

    scoringPaths.add(
        locationsToPath(descriptor.startingLocation(), descriptor.scoringLocations().get(0))
            .flipOnAlliance(flipAuto));

    for (int i = 1; i < descriptor.scoringLocations().size(); i++)
      scoringPaths.add(
          locationsToPath(
                  descriptor.coralStationLocations().get(i - 1),
                  descriptor.scoringLocations().get(i))
              .flipOnAlliance(flipAuto));

    for (int i = 1; i < descriptor.coralStationLocations().size(); i++)
      coralStationPaths.add(
          locationsToPath(
                  descriptor.scoringLocations().get(i), descriptor.coralStationLocations().get(i))
              .flipOnAlliance(flipAuto));

    this.config = config;

    setInitialState(stateWithName("PrepScorePathing", () -> prepScoreCoralPathing()));
  }

  // CORAL SCORING STATES

  private StateHandler prepScoreCoralPathing() {
    ChoreoTrajectoryWithName traj = scoringPaths.get(scoringIndex);
    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            wrapper.getPoseEstimatorPose(true),
            traj,
            config.getDriveBaseRadius() / 2,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());
    scoringEndPose = traj.getFinalPose(Constants.isRedAlliance());
    return stateWithName("ScorePathing", () -> scoreCoralPathing());
  }

  private StateHandler scoreCoralPathing() {
    Pose2d currentPose = wrapper.getPoseEstimatorPose(true);
    wrapper.setVelocityOverride(
        choreoHelper.calculateChassisSpeeds(currentPose, timeFromStart()).chassisSpeeds());
    if (GeometryUtil.getDist(currentPose, scoringEndPose) < distBeforeScoringMeters.get())
      return stateWithName("PrepScoreCoral", () -> prepScoreCoral());
    return null;
  }

  private StateHandler prepScoreCoral() {
    RobotStates.scoringLevel = scoringLocations.get(scoringIndex).level();
    return suspendForSubStateMachine(
        new ScoreCoral(
            wrapper,
            elevator,
            arm,
            endEffector,
            led,
            scoringLocations.get(scoringIndex).side(),
            (r) -> {}),
        (s) -> {
          scoringIndex++;
          return stateWithName("PrepIntakeCoral", () -> prepIntakeCoral());
        });
  }

  // CORAL INTAKING STATES

  private StateHandler prepIntakeCoral() {
    ChoreoTrajectoryWithName traj = coralStationPaths.get(intakingIndex);
    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            wrapper.getPoseEstimatorPose(true),
            traj,
            config.getDriveBaseRadius() / 2,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());
    spawnCommand(
        null,
        (c) -> {
          intakingIndex++;
          return stateWithName("PrepScoreCoralPathing", () -> prepScoreCoralPathing());
        });
    return stateWithName("IntakeCoral", () -> intakeCoral());
  }

  private StateHandler intakeCoral() {
    wrapper.setVelocityOverride(
        choreoHelper
            .calculateChassisSpeeds(wrapper.getPoseEstimatorPose(true), timeFromStart())
            .chassisSpeeds());
    return null;
  }

  // Additional Methods

  private ChoreoTrajectoryWithName locationsToPath(
      ScoringLocation scoring, CoralStationLocation coralStation) {
    return StringsToPath(scoring.toString(), coralStation.toString());
  }

  private ChoreoTrajectoryWithName locationsToPath(
      CoralStationLocation coralStation, ScoringLocation scoring) {
    return StringsToPath(coralStation.toString(), scoring.toString());
  }

  private ChoreoTrajectoryWithName locationsToPath(
      StartingLocation starting, ScoringLocation scoring) {
    return StringsToPath(starting.toString(), scoring.toString());
  }

  private ChoreoTrajectoryWithName StringsToPath(String startString, String endString) {
    String fullString = startString + "_" + endString;
    return ChoreoTrajectoryWithName.getTrajectory(fullString);
  }

  public Pose2d initPose() {
    return scoringPaths.get(0).getInitialPose(Constants.isRedAlliance());
  }
}
