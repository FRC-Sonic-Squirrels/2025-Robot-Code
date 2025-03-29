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
import frc.lib.team6328.GeomUtil;
import frc.robot.CommandComposer;
import frc.robot.Constants;
import frc.robot.FieldStates;
import frc.robot.RobotStates;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.helpers.ChoreoHelper.ChassisSpeedsWithPathEnd;
import frc.robot.autonomous.records.AutoDescriptor;
import frc.robot.autonomous.records.AutoDescriptor.StartingLocation;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.autonomous.records.OppositeSide;
import frc.robot.autonomous.records.PickupLocation;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.autonomous.records.ScoringLocation.ReefSide;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoreCoral.ScoreCoralObjective;
import frc.robot.commands.drive.DriveToPosePathing;
import frc.robot.commands.intake.IntakeCoralGround;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.mechanism.MechanismPositions;
import frc.robot.subsystems.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoStateMachine extends StateMachine {

  private final DrivetrainWrapper wrapper;
  private final Mechanism mech;
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endEffector;
  private final LED led;
  private final RobotStates states;
  private final Intake intake;

  private final List<ChoreoTrajectoryWithName> scoringPaths = new ArrayList<>();
  private final List<ScoringLocation> scoringLocations;
  private final List<ChoreoTrajectoryWithName> coralStationPaths = new ArrayList<>();
  private final List<PickupLocation> coralStationLocations;

  private final RobotConfig config;

  private int scoringIndex = 0;
  private int intakingIndex = 0;

  private ChoreoHelper choreoHelper;

  private static LoggerGroup logGroup = LoggerGroup.build("AutoStateMachine");

  private static LoggerEntry.Struct<Pose3d> log_usedCoralTag =
      logGroup.buildStruct(Pose3d.class, "UsedCoralTag");
  private static LoggerEntry.Bool log_foundPath = logGroup.buildBoolean("PathFound");

  private final boolean procedural;
  private Consumer<Double> rumble = null;

  private ScoreCoral scoreCoral;

  private static AutoDescriptor preloadCodeDescriptor() {
    List<ScoringLocation> scoringLocations = new ArrayList<>();
    List<PickupLocation> coralStationLocations = new ArrayList<>();

    scoringLocations.add(new ScoringLocation(ReefSide.CI, ScoringLevel.L4));
    coralStationLocations.add(PickupLocation.IA);

    scoringLocations.add(new ScoringLocation(ReefSide.CK, ScoringLevel.L4));
    coralStationLocations.add(PickupLocation.IA);

    scoringLocations.add(new ScoringLocation(ReefSide.CL, ScoringLevel.L4));
    coralStationLocations.add(PickupLocation.IA);

    scoringLocations.add(new ScoringLocation(ReefSide.CJ, ScoringLevel.L4));
    coralStationLocations.add(PickupLocation.IA);
    return new AutoDescriptor(scoringLocations, coralStationLocations, StartingLocation.S2);
  }

  private boolean preloadCode;

  /** IMPORTANT: Use this constructor only for preloading code */
  public AutoStateMachine(AutosSubsystems subsystems, RobotConfig config, RobotStates states) {
    this(subsystems, preloadCodeDescriptor(), config, false, false, states);
    preloadCode = true;
    advance();
  }

  public AutoStateMachine(
      AutosSubsystems subsystems, RobotConfig config, Consumer<Double> rumble, RobotStates states) {
    this(subsystems, null, config, false, true, states);
    this.rumble = rumble;
  }

  /** Creates a new AutoSubstateMachine. */
  public AutoStateMachine(
      AutosSubsystems subsystems,
      AutoDescriptor descriptor,
      RobotConfig config,
      boolean flipAuto,
      boolean procedural,
      RobotStates states) {
    super("Auto");

    wrapper = subsystems.drivetrain();
    mech = subsystems.mech();
    elevator = mech.getElevator();
    arm = mech.getArm();
    endEffector = subsystems.endEffector();
    led = subsystems.led();
    intake = subsystems.intake();
    this.states = states;

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
          locationsToPath(descriptor.startingLocation(), descriptor.scoringLocations().get(0)));

      for (int i = 1; i < descriptor.scoringLocations().size(); i++) {
        scoringPaths.add(
            locationsToPath(
                descriptor.coralStationLocations().get(i - 1),
                descriptor.scoringLocations().get(i)));
      }

      for (int i = 0; i < descriptor.coralStationLocations().size(); i++) {
        PickupLocation pickup = descriptor.coralStationLocations().get(i);
        coralStationPaths.add(locationsToPath(descriptor.scoringLocations().get(i), pickup));
      }
    }

    boolean foundPath = scoringPaths.indexOf(null) == -1 && coralStationPaths.indexOf(null) == -1;

    log_foundPath.info(foundPath);

    if (foundPath) {
      for (int i = 0; i < scoringPaths.size(); i++) {
        scoringPaths.set(i, scoringPaths.get(i).flipOnAlliance(flipAuto).flipForAlliance());
      }
      for (int i = 0; i < coralStationPaths.size(); i++) {
        coralStationPaths.set(
            i, coralStationPaths.get(i).flipOnAlliance(flipAuto).flipForAlliance());
      }
    }

    this.config = config;

    setInitialState(stateWithName("PrepScoreCoral", () -> prepScoreCoral()));

    preloadCode();
  }

  private void preloadCode() {
    // Pretend to create a path, to load all the pathing code.
    var cmd =
        new DriveToPosePathing(
            wrapper,
            config,
            () -> wrapper.getCoralStationPoseEstimatorPose(true),
            this::getClosestCoralStationPose);
    cmd.initializeNoCheck();
  }

  // CORAL SCORING STATES

  private StateHandler prepScoreCoral() {
    states.intakeState = IntakeState.Stow;
    if (scoringLocations != null && scoringIndex == scoringLocations.size()) {
      return stateWithName("Done", setDone());
    }

    if (scoringLocations == null) {

      scoreCoral =
          new ScoreCoral(
              wrapper,
              mech,
              elevator,
              arm,
              led,
              rumble,
              config,
              () -> ScoreCoralObjective.JustScore,
              states);

    } else {

      states.scoringLevel = scoringLocations.get(scoringIndex).level();
      scoreCoral =
          new ScoreCoral(
              wrapper,
              mech,
              elevator,
              arm,
              led,
              scoringLocations.get(scoringIndex).side(),
              (r) -> {},
              config,
              () -> ScoreCoralObjective.JustScore,
              procedural ? Optional.empty() : Optional.of(scoringPaths.get(scoringIndex)),
              preloadCode,
              states);
    }

    spawnStateMachineAsCommand(scoreCoral, (s) -> null, preloadCode);

    return stateWithName("ScoreCoral", () -> scoreCoral());
  }

  private StateHandler scoreCoral() {
    if (!scoreCoral.usingDrivetrain() || preloadCode) {
      var endEffectorSim = endEffector.getSim();
      if (endEffectorSim != null && !preloadCode) {
        endEffectorSim.scoringSideTofDetecting = false;
        endEffectorSim.nonScoringSideTofDetecting = false;
        FieldStates.setScoringLocationFilled(
            new ScoringLocation(states.targetReefSide, states.scoringLevel));
      }
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
              coralStationLocations.get(intakingIndex).ground
                  ? wrapper.getReefPoseEstimatorPose(true)
                  : wrapper.getCoralStationPoseEstimatorPose(true),
              traj,
              config.getDriveBaseRadius() / 2,
              config.getAutoTranslationPidController(),
              config.getAutoTranslationPidController(),
              config.getAutoThetaPidController());
    }
    Supplier<Pose2d> intakingPoseSupplier =
        () ->
            coralStationLocations == null
                ? getClosestCoralStationPose()
                : getCoralStationPose(coralStationLocations.get(intakingIndex));

    MechanismPosition coralStationPos = MechanismPositions.coralStationPosition();

    if (procedural) {
      spawnCommand(
          new DriveToPosePathing(
              wrapper,
              config,
              () -> wrapper.getCoralStationPoseEstimatorPose(true),
              intakingPoseSupplier),
          (c) -> null);
    }

    spawnCommand(
        coralStationLocations.get(intakingIndex).ground
            ? new IntakeCoralGround(intake, states)
            : CommandComposer.intakeCoralFromStation(
                wrapper, endEffector, mech, led, null, false, states),
        (c) -> null);

    return stateWithName("IntakeCoral", () -> intakeCoral());
  }

  private StateHandler intakeCoral() {
    if (!procedural) {
      ChassisSpeedsWithPathEnd result =
          choreoHelper.calculateChassisSpeeds(
              coralStationLocations.get(intakingIndex).ground
                  ? wrapper.getReefPoseEstimatorPose(true)
                  : wrapper.getCoralStationPoseEstimatorPose(true),
              timeFromStart());
      wrapper.setVelocityOverride(result.chassisSpeeds());
    }

    return states.coralInEndEffector || states.coralInIntake || preloadCode
        ? stateWithName("ReturnToScoring", () -> returnToScoring())
        : null;
  }

  private StateHandler returnToScoring() {
    intakingIndex++;
    return stateWithName("PrepScoreCoral", () -> prepScoreCoral());
  }

  // Additional Methods

  public static ChoreoTrajectoryWithName locationsToPath(
      ScoringLocation scoring, PickupLocation coralStation) {
    return enumsToPath(scoring.side(), coralStation);
  }

  public static ChoreoTrajectoryWithName locationsToPath(
      PickupLocation coralStation, ScoringLocation scoring) {
    return enumsToPath(coralStation, scoring.side());
  }

  public static ChoreoTrajectoryWithName locationsToPath(
      StartingLocation starting, ScoringLocation scoring) {
    return enumsToPath(starting, scoring.side());
  }

  private static <T1, T2> ChoreoTrajectoryWithName enumsToPath(
      OppositeSide<T1> starting, OppositeSide<T2> ending) {
    ChoreoTrajectoryWithName traj = stringsToPathSimple(starting, ending);
    if (traj == null) {
      var oppositeStarting = starting.getOpposite();
      var oppositeEnding = ending.getOpposite();
      traj = stringsToPathSimple(oppositeStarting, oppositeEnding);
      if (traj == null) {
        throw new RuntimeException("Could not find " + starting + "_" + ending);
      }
      traj = traj.flipOnAlliance(true);
    }

    return traj;
  }

  private static ChoreoTrajectoryWithName stringsToPathSimple(
      OppositeSide<?> startString, OppositeSide<?> endString) {
    String fullString = startString.toString() + "_" + endString.toString();
    return ChoreoTrajectoryWithName.getTrajectory(fullString);
  }

  public Pose2d initPose() {
    ChoreoTrajectoryWithName traj = scoringPaths.get(0);
    if (traj == null) return null;
    return traj.getInitialPose(false);
  }

  private Pose2d getCoralStationPose(PickupLocation location) {
    boolean top = location == PickupLocation.IA || location == PickupLocation.IB;
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

    boolean left = location == PickupLocation.IA || location == PickupLocation.IC;

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

    for (PickupLocation location : PickupLocation.values()) {
      Pose2d trialPose = getCoralStationPose(location);
      if (bestPose == null
          || GeometryUtil.getDist(trialPose, wrapper.getCoralStationPoseEstimatorPose(true))
              < GeometryUtil.getDist(bestPose, wrapper.getCoralStationPoseEstimatorPose(true))) {
        bestPose = trialPose;
      }
    }

    return bestPose;
  }
}
