// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.GeomUtil;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.RobotStates;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.helpers.ChoreoHelper.ChassisSpeedsWithPathEnd;
import frc.robot.autonomous.records.AutoDescriptor;
import frc.robot.autonomous.records.AutoDescriptor.StartingLocation;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.autonomous.records.CoralStationLocation;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.drive.DriveToPosePathing;
import frc.robot.commands.endEffector.IntakeGamepieceCoralStation;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoStateMachine extends StateMachine {

  private final DrivetrainWrapper wrapper;
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endEffector;
  private final LED led;

  private final List<ChoreoTrajectoryWithName> scoringPaths = new ArrayList<>();
  private final List<ScoringLocation> scoringLocations;
  private final List<ChoreoTrajectoryWithName> coralStationPaths = new ArrayList<>();
  private final List<CoralStationLocation> coralStationLocations;

  private final RobotConfig config;

  private int scoringIndex = 0;
  private int intakingIndex = 0;

  private ChoreoHelper choreoHelper;

  private Pose2d scoringEndPose = Pose2d.kZero;

  private TunableNumberGroup group = new TunableNumberGroup("AutoStateMachine");
  private LoggedTunableNumber distBeforeScoringMeters = group.build("DistBeforeScoringMeters", 1);

  private LoggerGroup logGroup = LoggerGroup.build("AutoStateMachine");

  private LoggerEntry.Struct<Pose3d> log_usedCoralTag =
      logGroup.buildStruct(Pose3d.class, "UsedCoralTag");

  private final boolean procedural;
  private Consumer<Double> rumble = null;

  private ScoreCoral scoreCoral;

  public AutoStateMachine(AutosSubsystems subsystems, RobotConfig config, Consumer<Double> rumble) {
    this(subsystems, null, config, false, true);
    this.rumble = rumble;
  }

  /** Creates a new AutoSubstateMachine. */
  public AutoStateMachine(
      AutosSubsystems subsystems,
      AutoDescriptor descriptor,
      RobotConfig config,
      boolean flipAuto,
      boolean procedural) {
    super("Auto");

    wrapper = subsystems.drivetrain();
    elevator = subsystems.elevator();
    arm = subsystems.arm();
    endEffector = subsystems.endEffector();
    led = subsystems.led();

    if (descriptor == null) {
      scoringLocations = null;
      coralStationLocations = null;
    } else {
      scoringLocations =
          flipAuto ? descriptor.flippedScoringLocations() : descriptor.scoringLocations();
      coralStationLocations =
          flipAuto ? descriptor.flippedCoralStationLocations() : descriptor.coralStationLocations();
    }

    this.procedural = procedural;

    if (!procedural) {
      scoringPaths.add(
          locationsToPath(descriptor.startingLocation(), descriptor.scoringLocations().get(0))
              .flipOnAlliance(flipAuto));

      for (int i = 1; i < descriptor.scoringLocations().size(); i++)
        scoringPaths.add(
            locationsToPath(
                    descriptor.coralStationLocations().get(i - 1),
                    descriptor.scoringLocations().get(i))
                .flipOnAlliance(flipAuto));

      for (int i = 0; i < descriptor.coralStationLocations().size(); i++)
        coralStationPaths.add(
            locationsToPath(
                    descriptor.scoringLocations().get(i), descriptor.coralStationLocations().get(i))
                .flipOnAlliance(flipAuto));
    }

    this.config = config;

    setInitialState(stateWithName("PrepScorePathing", () -> prepScoreCoralPathing()));
  }

  // CORAL SCORING STATES

  private StateHandler prepScoreCoralPathing() {
    if (scoringLocations != null && scoringIndex == scoringLocations.size()) {
      return stateWithName("Done", setDone());
    }

    if (procedural) return stateWithName("PrepScoreCoral", () -> prepScoreCoral());

    ChoreoTrajectoryWithName traj = scoringPaths.get(scoringIndex);
    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            wrapper.getReefPoseEstimatorPose(true),
            traj,
            config.getDriveBaseRadius() / 2,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());
    scoringEndPose = traj.getFinalPose(Constants.isRedAlliance());
    return stateWithName("ScorePathing", () -> scoreCoralPathing());
  }

  private StateHandler scoreCoralPathing() {
    Pose2d currentPose = wrapper.getReefPoseEstimatorPose(true);
    wrapper.setVelocityOverride(
        choreoHelper.calculateChassisSpeeds(currentPose, timeFromStart()).chassisSpeeds());
    if (GeometryUtil.getDist(currentPose, scoringEndPose) < distBeforeScoringMeters.get())
      return stateWithName("PrepScoreCoral", () -> prepScoreCoral());
    return null;
  }

  private StateHandler prepScoreCoral() {

    if (scoringLocations == null) {

      scoreCoral = new ScoreCoral(wrapper, elevator, arm, endEffector, led, rumble, config);

    } else {

      RobotStates.scoringLevel = scoringLocations.get(scoringIndex).level();
      scoreCoral =
          new ScoreCoral(
              wrapper,
              elevator,
              arm,
              endEffector,
              led,
              scoringLocations.get(scoringIndex).side(),
              (r) -> {},
              config);
    }

    spawnStateMachineAsCommand(scoreCoral, (s) -> null);

    return stateWithName("ScoreCoral", () -> scoreCoral());
  }

  private StateHandler scoreCoral() {
    if (!scoreCoral.usingDrivetrain()) {
      scoringIndex++;
      return stateWithName("PrepIntakeCoral", () -> prepIntakeCoral());
    }

    return null;
  }

  // CORAL INTAKING STATES

  private StateHandler prepIntakeCoral() {
    if (coralStationLocations != null && intakingIndex == coralStationLocations.size()) {
      return stateWithName("Done", setDone());
    }

    if (!procedural) {
      ChoreoTrajectoryWithName traj = coralStationPaths.get(intakingIndex);
      choreoHelper =
          new ChoreoHelper(
              timeFromStart(),
              wrapper.getCoralStationPoseEstimatorPose(true),
              traj,
              config.getDriveBaseRadius() / 2,
              config.getAutoTranslationPidController(),
              config.getAutoTranslationPidController(),
              config.getAutoThetaPidController());
    }
    spawnCommand(
        new IntakeGamepieceCoralStation(
            endEffector, () -> wrapper.getCoralStationPoseEstimatorPose(true)),
        (c) -> {
          intakingIndex++;
          return stateWithName("PrepScoreCoralPathing", () -> prepScoreCoralPathing());
        });
    Supplier<Pose2d> intakingPoseSupplier =
        () ->
            coralStationLocations == null
                ? getClosestCoralStationPose()
                : getCoralStationPose(coralStationLocations.get(intakingIndex));
    return procedural
        ? suspendForCommand(
            new DriveToPosePathing(
                wrapper,
                config,
                () -> wrapper.getCoralStationPoseEstimatorPose(true),
                intakingPoseSupplier),
            (c) -> stateWithName("NotifySimPathEnded", () -> notifySimPathEnded()))
        : stateWithName("IntakeCoral", () -> intakeCoral());
  }

  private StateHandler notifySimPathEnded() {

    if (RobotMode.isSimBot()) {
      RobotStates.coralInEndEffectorNonScoringSide = true;
    }

    return null;
  }

  private StateHandler intakeCoral() {
    ChassisSpeedsWithPathEnd result =
        choreoHelper.calculateChassisSpeeds(
            wrapper.getReefPoseEstimatorPose(true), timeFromStart());
    wrapper.setVelocityOverride(result.chassisSpeeds());

    return null;
  }

  // Additional Methods

  private ChoreoTrajectoryWithName locationsToPath(
      ScoringLocation scoring, CoralStationLocation coralStation) {
    return StringsToPath(scoring.side().toString(), coralStation.toString());
  }

  private ChoreoTrajectoryWithName locationsToPath(
      CoralStationLocation coralStation, ScoringLocation scoring) {
    return StringsToPath(coralStation.toString(), scoring.side().toString());
  }

  private ChoreoTrajectoryWithName locationsToPath(
      StartingLocation starting, ScoringLocation scoring) {
    return StringsToPath(starting.toString(), scoring.side().toString());
  }

  private ChoreoTrajectoryWithName StringsToPath(String startString, String endString) {
    String fullString = startString + "_" + endString;
    return ChoreoTrajectoryWithName.getTrajectory(fullString);
  }

  public Pose2d initPose() {
    return scoringPaths.get(0).getInitialPose(false);
  }

  private Pose2d getCoralStationPose(CoralStationLocation location) {
    boolean top = location == CoralStationLocation.IA || location == CoralStationLocation.IB;
    Pose3d tagPose = config.getAprilTagFieldLayout().getTagPose(top ? 13 : 12).get();
    log_usedCoralTag.info(tagPose);
    Pose2d pose = tagPose.toPose2d();
    Pose2d centerPickup =
        new Pose2d(
            pose.getTranslation()
                .plus(
                    new Translation2d(
                        Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getX() / 2.0,
                        pose.getRotation())),
            pose.getRotation().plus(Rotation2d.k180deg));

    boolean left = location == CoralStationLocation.IA || location == CoralStationLocation.IC;

    Pose2d offsetPickup =
        centerPickup.plus(
            GeomUtil.translationToTransform(
                new Translation2d(
                    Constants.FieldConstants.CORAL_STATION_WIDTH.div(4).in(Units.Meter),
                    left ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg)));
    return AllianceFlipUtil.flipPoseForAlliance(offsetPickup);
  }

  private Pose2d getClosestCoralStationPose() {
    Pose2d bestPose = null;

    for (CoralStationLocation location : CoralStationLocation.values()) {
      Pose2d trialPose = getCoralStationPose(location);
      if (bestPose == null
          || GeometryUtil.getDist(trialPose, wrapper.getCoralStationPoseEstimatorPose(true))
              < GeometryUtil.getDist(bestPose, wrapper.getCoralStationPoseEstimatorPose(true)))
        bestPose = trialPose;
    }

    return bestPose;
  }
}
