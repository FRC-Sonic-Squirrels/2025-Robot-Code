// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.helpers.ChoreoHelper.ChassisSpeedsWithPathEnd;
import frc.robot.autonomous.records.AutoDescriptor;
import frc.robot.autonomous.records.AutoDescriptor.StartingLocation;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.autonomous.records.CoralStationLocation;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.drive.DriveToPosePathing;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.endEffector.EndEffector;
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

  private final List<ChoreoTrajectoryWithName> scoringPaths = new ArrayList<>();
  private final List<ScoringLocation> scoringLocations;
  private final List<ChoreoTrajectoryWithName> coralStationPaths = new ArrayList<>();
  private final List<CoralStationLocation> coralStationLocations;

  private final RobotConfig config;

  private int scoringIndex = 0;
  private int intakingIndex = 0;

  private ChoreoHelper choreoHelper;

  private LoggerGroup logGroup = LoggerGroup.build("AutoStateMachine");

  private LoggerEntry.Struct<Pose3d> log_usedCoralTag =
      logGroup.buildStruct(Pose3d.class, "UsedCoralTag");
  private LoggerEntry.Bool log_foundPath = logGroup.buildBoolean("PathFound");
  private LoggerEntry.Text log_missingPath = logGroup.buildString("MissingPath");

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
    mech = subsystems.mech();
    elevator = mech.getElevator();
    arm = mech.getArm();
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
          locationsToPath(descriptor.startingLocation(), descriptor.scoringLocations().get(0)));

      for (int i = 1; i < descriptor.scoringLocations().size(); i++)
        scoringPaths.add(
            locationsToPath(
                descriptor.coralStationLocations().get(i - 1),
                descriptor.scoringLocations().get(i)));

      for (int i = 0; i < descriptor.coralStationLocations().size(); i++)
        coralStationPaths.add(
            locationsToPath(
                descriptor.scoringLocations().get(i), descriptor.coralStationLocations().get(i)));
    }

    boolean foundPath = scoringPaths.indexOf(null) == -1 && coralStationPaths.indexOf(null) == -1;

    log_foundPath.info(foundPath);

    if (foundPath) {
      for (int i = 0; i < scoringPaths.size(); i++)
        scoringPaths.set(i, scoringPaths.get(i).flipOnAlliance(flipAuto).flipForAlliance());
      for (int i = 0; i < coralStationPaths.size(); i++)
        coralStationPaths.set(
            i, coralStationPaths.get(i).flipOnAlliance(flipAuto).flipForAlliance());
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
    if (scoringLocations != null && scoringIndex == scoringLocations.size()) {
      return stateWithName("Done", setDone());
    }

    if (scoringLocations == null) {

      scoreCoral = new ScoreCoral(wrapper, mech, elevator, arm, led, rumble, config, false);

    } else {

      RobotStates.scoringLevel = scoringLocations.get(scoringIndex).level();
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
              false,
              procedural ? Optional.empty() : Optional.of(scoringPaths.get(scoringIndex)));
    }

    spawnStateMachineAsCommand(scoreCoral, (s) -> null);

    return stateWithName("ScoreCoral", () -> scoreCoral());
  }

  private StateHandler scoreCoral() {
    if (!scoreCoral.usingDrivetrain()) {
      var endEffectorSim = endEffector.getSim();
      if (endEffectorSim != null) {
        endEffectorSim.scoringSideTofDetecting = false;
        endEffectorSim.nonScoringSideTofDetecting = false;
        FieldStates.setScoringLocationFilled(
            new ScoringLocation(RobotStates.targetReefSide, RobotStates.scoringLevel));
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
              wrapper.getCoralStationPoseEstimatorPose(true),
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

    if (procedural)
      spawnCommand(
          new DriveToPosePathing(
              wrapper,
              config,
              () -> wrapper.getCoralStationPoseEstimatorPose(true),
              intakingPoseSupplier),
          (c) -> null);

    spawnCommand(
        Commands.waitUntil(
                () ->
                    elevator.isAtTarget(coralStationPos.elevatorHeight())
                        && arm.isAtTargetAngle(coralStationPos.armAngle()))
            .andThen(
                CommandComposer.intakeCoralFromStation(wrapper, endEffector, mech, led, null, false)
                    .asProxy()),
        (c) -> null);

    return stateWithName("IntakeCoral", () -> intakeCoral());
  }

  private StateHandler intakeCoral() {
    if (!procedural) {
      ChassisSpeedsWithPathEnd result =
          choreoHelper.calculateChassisSpeeds(
              wrapper.getCoralStationPoseEstimatorPose(true), timeFromStart());
      wrapper.setVelocityOverride(result.chassisSpeeds());
    }

    return RobotStates.coralInEndEffector
        ? stateWithName("ReturnToScoring", () -> returnToScoring())
        : null;
  }

  private StateHandler returnToScoring() {
    intakingIndex++;
    return stateWithName("PrepScoreCoral", () -> prepScoreCoral());
  }

  // Additional Methods

  private ChoreoTrajectoryWithName locationsToPath(
      ScoringLocation scoring, CoralStationLocation coralStation) {
    return stringsToPath(scoring.side().toString(), coralStation.toString());
  }

  private ChoreoTrajectoryWithName locationsToPath(
      CoralStationLocation coralStation, ScoringLocation scoring) {
    return stringsToPath(coralStation.toString(), scoring.side().toString());
  }

  private ChoreoTrajectoryWithName locationsToPath(
      StartingLocation starting, ScoringLocation scoring) {
    return stringsToPath(starting.toString(), scoring.side().toString());
  }

  private ChoreoTrajectoryWithName stringsToPath(String startString, String endString) {
    ChoreoTrajectoryWithName traj = stringsToPathSimple(startString, endString);
    if (traj == null) {
      String oppositeStartString = oppositeLocation(startString);
      String oppositeEndString = oppositeLocation(endString);
      traj = stringsToPathSimple(oppositeStartString, oppositeEndString);
      if (traj == null) {
        log_missingPath.info(
            startString + "_" + endString + " or " + oppositeStartString + "_" + oppositeEndString);

      } else {
        traj = traj.flipOnAlliance(true);
      }
    }
    return traj;
  }

  private ChoreoTrajectoryWithName stringsToPathSimple(String startString, String endString) {
    String fullString = startString + "_" + endString;
    return ChoreoTrajectoryWithName.getTrajectory(fullString);
  }

  private String oppositeLocation(String location) {
    return switch (location) {
      case "S1" -> "S6";
      case "S2" -> "S5";
      case "S3" -> "S4";
      case "CA" -> "CB";
      case "CH" -> "CG";
      case "CI" -> "CF";
      case "CJ" -> "CE";
      case "CK" -> "CD";
      case "CL" -> "CC";
      case "IA" -> "ID";
      case "IB" -> "IC";
      case "S6" -> "S1";
      case "S5" -> "S2";
      case "S4" -> "S3";
      case "CB" -> "CA";
      case "CG" -> "CH";
      case "CF" -> "CI";
      case "CE" -> "CJ";
      case "CD" -> "CK";
      case "CC" -> "CL";
      case "ID" -> "IA";
      case "IC" -> "IB";
      default -> "";
    };
  }

  public Pose2d initPose() {
    if (scoringPaths.get(0) == null) return null;
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
