package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team2930.lib.controller_rumble.ControllerRumbleForTime;
import frc.lib.team6328.GeomUtil;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ScoringSideWithPose;
import frc.robot.Constants.FieldConstants.ScoringSideWithPoseAndDirection;
import frc.robot.Constants.RobotMode;
import frc.robot.FieldStates;
import frc.robot.RobotStates;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.autonomous.records.ScoringLocation.ReefSide;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.DriveToPosePathing;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.LED.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.Optional;
import java.util.function.Consumer;

public class ScoreCoral extends StateMachine {

  private final DrivetrainWrapper wrapper;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final EndEffector endEffector;
  private final LED led;
  private final RobotConfig config;

  private final Optional<ScoringDirection> optionalScoringDirection;
  private ScoringDirection scoringDirection;
  private ScoringSide scoringSide;
  private final Optional<ScoringSide> optionalScoringSide;
  private ScoringSideWithPoseAndDirection scoringPoseSideAndDirection;
  private Pose2d algaeClearPose;
  private Pose2d scoringPose;

  private Command prepMechanismForAlgae;
  private Command prepMechanismForScoring;

  private Command clearAlgae1Position;
  private Command clearAlgae2Position;

  private boolean usingDrivetrain = true;

  private final Consumer<Double> rumble;

  private boolean gamepieceMemory = false;

  private static final TunableNumberGroup group = new TunableNumberGroup("ScoreCoral");
  private static final LoggedTunableNumber distToRaiseMech =
      group.build("DistToRaiseMechMeters", 2);
  private static final LoggedTunableNumber scoringVelocityRPM =
      group.build("ScoringVelocityRPM", 1000);
  private static final LoggedTunableNumber predictiveTime =
      group.build("PredictiveTimeSeconds", 0.5);

  private static final LoggerGroup log_group = LoggerGroup.build("ScoreCoral");
  private static final LoggerEntry.EnumValue<ScoringSide> log_scoringSide =
      log_group.buildEnum("ScoringSide");
  private static final LoggerEntry.EnumValue<ScoringDirection> log_scoringDirection =
      log_group.buildEnum("ScoringDirection");
  private static final LoggerEntry.Struct<Pose2d> log_scoringPose =
      log_group.buildStruct(Pose2d.class, "ScoringPose");
  private static final LoggerEntry.Struct<Pose2d> log_algaeClearPose =
      log_group.buildStruct(Pose2d.class, "ScoringPose");
  private static final LoggerEntry.Struct<Pose2d> log_predictedPose =
      log_group.buildStruct(Pose2d.class, "PosePrediction/PredictedPose");
  private static final LoggerEntry.Struct<Translation2d> log_inputVel =
      log_group.buildStruct(Translation2d.class, "PosePrediction/InputVel");
  private static final LoggerEntry.Struct<Translation2d> log_adjustedVel =
      log_group.buildStruct(Translation2d.class, "PosePrediction/AdjustedVel");
  private static final LoggerEntry.Decimal log_headingToReef =
      log_group.buildDecimal("PosePrediction/HeadingToReef");
  private static final LoggerEntry.Decimal log_deltaTheta =
      log_group.buildDecimal("PosePrediction/DeltaTheta");

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Elevator elevator,
      Arm arm,
      EndEffector endEffector,
      LED led,
      Consumer<Double> rumble,
      RobotConfig config) {
    this(
        wrapper,
        elevator,
        arm,
        endEffector,
        led,
        Optional.empty(),
        Optional.empty(),
        rumble,
        config);
    gamepieceMemory = true;
  }

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Elevator elevator,
      Arm arm,
      Intake intake,
      EndEffector endEffector,
      LED led,
      Consumer<Double> rumble,
      RobotConfig config) {
    this(
        wrapper,
        elevator,
        arm,
        intake,
        endEffector,
        led,
        reefSideToScoringDirection(side),
        Optional.of(reefSideToScoringSide(side)),
        rumble,
        config);
    gamepieceMemory = true;
  }

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Elevator elevator,
      Arm arm,
      Intake intake,
      EndEffector endEffector,
      LED led,
      ScoringDirection scoringDirection,
      Consumer<Double> rumble,
      RobotConfig config) {
    this(
        wrapper,
        elevator,
        arm,
        endEffector,
        led,
        Optional.of(scoringDirection),
        Optional.empty(),
        rumble,
        config);
  }

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Elevator elevator,
      Arm arm,
      Intake intake,
      EndEffector endEffector,
      LED led,
      Optional<ScoringDirection> scoringDirection,
      Optional<ScoringSide> side,
      Consumer<Double> rumble,
      RobotConfig config) {
    super("ScoreCoral");

    this.wrapper = wrapper;
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.endEffector = endEffector;
    this.led = led;
    this.config = config;

    this.optionalScoringDirection = scoringDirection;
    this.rumble = rumble;
    this.optionalScoringSide = side;

    setInterruptedState(stateWithName("End", () -> end(true)));
    setInitialState(stateWithName("PrepForScoringAlignment", () -> prepForScoringAlignment()));
  }

  // SCORING STATES

  private StateHandler prepForScoringAlignment() {

    Pose2d robotPose = wrapper.getReefPoseEstimatorPose(true);

    scoringPoseSideAndDirection =
        optionalScoringSide.isEmpty()
            ? getOptimalScoringSide(
                robotPose, wrapper.getFieldRelativeVelocities().getTranslation())
            : getScoringSide(optionalScoringSide.get());

    scoringPose = scoringPoseSideAndDirection.pose();

    scoringSide = scoringPoseSideAndDirection.side();

    scoringDirection = scoringPoseSideAndDirection.direction();

    log_scoringSide.info(scoringSide);
    log_scoringDirection.info(scoringDirection);
    log_scoringPose.info(scoringPose);
    log_algaeClearPose.info(scoringPoseSideAndDirection.pose());

    led.setBaseRobotState(BaseRobotState.SCORING_ALIGNMENT);

    if (!RobotStates.coralInEndEffector || !scorableLevel()) {
      return RobotStates.clearingAlgae && !gamepieceMemory
          ? stateWithName("PrepForAlgaeAlignment", () -> prepForAlgaeAlignment())
          : stateWithName("ScoreFailure", () -> scoreFailure());
    }

    prepMechanismForScoring =
        spawnCommand(
            Commands.waitUntil(
                    () ->
                        GeometryUtil.getDist(wrapper.getReefPoseEstimatorPose(true), scoringPose)
                            < distToRaiseMech.get())
                .andThen(MechanismActions.reefPosition(elevator, arm, RobotStates.scoringLevel)),
            (command) -> null);

    return suspendForCommand(
        // new DriveToPose(wrapper, () -> scoringPose, () -> wrapper.getReefPoseEstimatorPose(true))
        new DriveToPosePathing(
            wrapper, config, () -> wrapper.getReefPoseEstimatorPose(true), () -> scoringPose),
        (command) -> stateWithName("Score", () -> score()));
  }

  private StateHandler score() {

    if (!prepMechanismForScoring.isScheduled()) {
      endEffector.setVelocity(scoringVelocityRPM.get());
      if (RobotMode.isSimBot()) {
        RobotStates.coralInEndEffectorNonScoringSide = false;
        RobotStates.coralInEndEffectorScoringSide = false;
      }
    }

    if (RobotStates.coralInEndEffector) return null;

    FieldStates.setScoringLocationFilled(
        new ScoringLocation(
            scoringSideAndDirectionToReefSide(scoringSide, scoringDirection),
            RobotStates.scoringLevel));

    spawnCommand(new ControllerRumbleForTime(rumble, 0.25, 0.3), (c) -> null);

    led.setRobotState(RobotState.SCORE_SUCCESS);

    return RobotStates.clearingAlgae
            && !(gamepieceMemory && !FieldStates.isAlgaeInScoringSide(scoringSide))
        ? stateWithName("PrepForAlgaeAlignment", () -> prepForAlgaeAlignment())
        : stateWithName("End", () -> end(false));
  }

  // ALGAE STATES

  private StateHandler prepForAlgaeAlignment() {
    led.setBaseRobotState(BaseRobotState.ALGAE_ALIGNMENT);
    boolean high =
        scoringSide == ScoringSide.NEAR_MID
            || scoringSide == ScoringSide.FAR_LEFT
            || scoringSide == ScoringSide.FAR_RIGHT;

    clearAlgae1Position =
        high
            ? MechanismActions.clearAlgaeHigh1Position(elevator, arm, intake)
            : MechanismActions.clearAlgaeLow1Position(elevator, arm, intake);
    clearAlgae2Position =
        high
            ? MechanismActions.clearAlgaeHigh2Position(elevator, arm, intake)
            : MechanismActions.clearAlgaeLow2Position(elevator, arm, intake);

    prepMechanismForAlgae =
        spawnCommand(
            Commands.waitUntil(
                    () ->
                        GeometryUtil.getDist(wrapper.getReefPoseEstimatorPose(true), scoringPose)
                            < distToRaiseMech.get())
                .andThen(clearAlgae1Position),
            (command) -> null);

    algaeClearPose = getClosestAlgaeClearingSide(scoringPose).pose();

    DriveToPose driveToAlgaePose =
        new DriveToPose(
            wrapper, () -> algaeClearPose, () -> wrapper.getReefPoseEstimatorPose(true));

    return suspendForCommand(
        driveToAlgaePose,
        (command) ->
            suspendForCommand(
                Commands.waitUntil(
                        () ->
                            !prepMechanismForAlgae.isScheduled() && !driveToAlgaePose.isScheduled())
                    .andThen(clearAlgae2Position)
                    .andThen(
                        Commands.runOnce(
                            () -> FieldStates.removeAlgaeFromScoringSide(scoringSide))),
                (c) -> stateWithName("End", () -> end(false))));
  }

  private StateHandler scoreFailure() {
    spawnCommand(new ControllerRumbleForTime(rumble, 0.25, 0.3), (c) -> null);
    led.setRobotState(RobotState.SCORE_FAILURE);
    return stateWithName("End", () -> end(true));
  }

  private StateHandler end(boolean interrupted) {
    led.setBaseRobotState(BaseRobotState.GAMEPIECE_STATUS);
    usingDrivetrain = false;
    endEffector.setPercentOut(0);
    return suspendForCommand(
        MechanismActions.coralStationPosition(elevator, arm), (command) -> setDone());
  }

  private ScoringSideWithPoseAndDirection getScoringSide(ScoringSide side) {
    for (ScoringSideWithPoseAndDirection scoringLocation : getScoringLocations()) {
      if (scoringLocation.side() == side) {
        return scoringLocation;
      }
    }
    return null;
  }

  private ScoringSideWithPoseAndDirection getOptimalScoringSide(
      Pose2d robotPose, Translation2d vel) {

    log_inputVel.info(vel);

    Translation2d flippedReef =
        AllianceFlipUtil.flipTranslationForAlliance(Constants.FieldConstants.BLUE_REEF_CENTER_POSE);
    Rotation2d headingToReef =
        GeometryUtil.getHeading(robotPose.getTranslation(), flippedReef).unaryMinus();

    log_headingToReef.info(headingToReef);

    Translation2d adjustedVel = vel.rotateBy(headingToReef);

    log_adjustedVel.info(adjustedVel);

    Translation2d deltaPos = adjustedVel.times(predictiveTime.get());
    double distToReef = GeometryUtil.getDist(robotPose.getTranslation(), flippedReef);
    double newDistToReef =
        Math.max(
            distToReef - deltaPos.getX(),
            Constants.FieldConstants.REEF_WIDTH.in(Units.Meter) / 2.0);
    double avgRadius = (distToReef + newDistToReef) / 2.0;
    Rotation2d deltaTheta = Rotation2d.fromRadians(-deltaPos.getY() / avgRadius);
    Translation2d newTranslation =
        flippedReef.plus(
            new Translation2d(
                newDistToReef,
                GeometryUtil.getHeading(flippedReef, robotPose.getTranslation()).plus(deltaTheta)));

    log_predictedPose.info(GeomUtil.translationToPose(newTranslation));
    log_deltaTheta.info(deltaTheta);

    return getClosestScoringSide(newTranslation);
  }

  private ScoringSideWithPoseAndDirection getClosestScoringSide(Translation2d robotTranslation) {
    ScoringSideWithPoseAndDirection bestTarget =
        new ScoringSideWithPoseAndDirection(
            new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, Rotation2d.kZero),
            ScoringSide.FAR_LEFT,
            ScoringDirection.LEFT);

    for (ScoringSideWithPoseAndDirection pose : getScoringLocations()) {
      if (scoreableLocation(
              new ScoringLocation(
                  scoringSideAndDirectionToReefSide(pose.side(), pose.direction()),
                  RobotStates.scoringLevel))
          && GeometryUtil.getDist(robotTranslation, pose.pose().getTranslation())
              < GeometryUtil.getDist(robotTranslation, bestTarget.pose().getTranslation())) {
        bestTarget = pose;
      }
    }

    return bestTarget;
  }

  private ScoringSideWithPose getClosestAlgaeClearingSide(Pose2d robotPose) {
    ScoringSideWithPose bestTarget =
        new ScoringSideWithPose(
            new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, Rotation2d.kZero), ScoringSide.FAR_LEFT);
    for (ScoringSideWithPose pose : Constants.FieldConstants.SCORING_SIDES()) {
      if (GeometryUtil.getDist(robotPose, pose.pose())
          < GeometryUtil.getDist(robotPose, bestTarget.pose())) {
        bestTarget = pose;
      }
    }
    return bestTarget;
  }

  private ScoringSideWithPoseAndDirection[] getScoringLocations() {

    var sides = Constants.FieldConstants.SCORING_SIDES();

    var factor = optionalScoringDirection.isPresent() ? 1 : 2;

    var newSides = new ScoringSideWithPoseAndDirection[sides.length * factor];

    for (int i = 0; i < sides.length * factor; i++) {
      var scoringSidePose = sides[i / factor].pose();

      var scoringSide = sides[i / factor].side();

      Rotation2d objectiveScoringDirection;
      ScoringDirection direction;

      if (optionalScoringDirection.isPresent()) {
        direction = optionalScoringDirection.orElseThrow();
      } else {
        direction = i % 2 == 0 ? ScoringDirection.LEFT : ScoringDirection.RIGHT;
      }

      objectiveScoringDirection =
          direction == ScoringDirection.LEFT ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;

      Translation2d offset =
          new Translation2d(
              Constants.FieldConstants.REEF_BRANCH_OFFSET.in(Units.Meters),
              scoringSidePose.getRotation().plus(objectiveScoringDirection));
      Translation2d translation = scoringSidePose.getTranslation().plus(offset);
      newSides[i] =
          new ScoringSideWithPoseAndDirection(
              new Pose2d(translation, scoringSidePose.getRotation()), scoringSide, direction);
    }

    return newSides;
  }

  private static ScoringDirection reefSideToScoringDirection(ReefSide side) {
    return (side.ordinal() % 2 == 0) ? ScoringDirection.LEFT : ScoringDirection.RIGHT;
  }

  private static ScoringSide reefSideToScoringSide(ReefSide side) {
    return switch (side) {
      case CA, CB -> ScoringSide.NEAR_MID;
      case CC, CD -> ScoringSide.NEAR_RIGHT;
      case CE, CF -> ScoringSide.FAR_RIGHT;
      case CG, CH -> ScoringSide.FAR_MID;
      case CI, CJ -> ScoringSide.FAR_LEFT;
      case CK, CL -> ScoringSide.NEAR_LEFT;
    };
  }

  private static ReefSide scoringSideAndDirectionToReefSide(
      ScoringSide side, ScoringDirection direction) {
    switch (side) {
      case NEAR_MID:
        return direction == ScoringDirection.LEFT ? ReefSide.CA : ReefSide.CB;
      case NEAR_RIGHT:
        return direction == ScoringDirection.LEFT ? ReefSide.CC : ReefSide.CD;
      case FAR_RIGHT:
        return direction == ScoringDirection.LEFT ? ReefSide.CE : ReefSide.CF;
      case FAR_MID:
        return direction == ScoringDirection.LEFT ? ReefSide.CG : ReefSide.CH;
      case FAR_LEFT:
        return direction == ScoringDirection.LEFT ? ReefSide.CI : ReefSide.CJ;
      case NEAR_LEFT:
        return direction == ScoringDirection.LEFT ? ReefSide.CK : ReefSide.CL;
      default:
        return null;
    }
  }

  public boolean usingDrivetrain() {
    return usingDrivetrain;
  }

  private boolean conflictsWithAlgae(ScoringLocation location) {

    ScoringLevel level = location.level();

    ScoringSide side = reefSideToScoringSide(location.side());

    if (level == ScoringLevel.L3 && FieldStates.isAlgaeInScoringSide(side)) return true;

    if (level == ScoringLevel.L2
        && side.hasAlgaeAtStartOnL2
        && FieldStates.isAlgaeInScoringSide(side)) {
      return true;
    }

    return false;
  }

  private boolean scoreableLocation(ScoringLocation location) {
    return (!gamepieceMemory
        || !(FieldStates.isScoringLocationFilled(
                new ScoringLocation(location.side(), RobotStates.scoringLevel))
            || conflictsWithAlgae(location)));
  }

  private boolean scorableLevel() {
    for (ScoringSideWithPoseAndDirection pose : getScoringLocations()) {
      if (scoreableLocation(
          new ScoringLocation(
              scoringSideAndDirectionToReefSide(pose.side(), pose.direction()),
              RobotStates.scoringLevel))) return true;
    }

    return false;
  }

  public enum ScoringDirection {
    LEFT,
    RIGHT
  }

  public enum ScoringSide {
    NEAR_LEFT(true),
    NEAR_MID(false),
    NEAR_RIGHT(true),
    FAR_RIGHT(false),
    FAR_MID(true),
    FAR_LEFT(false);

    public final boolean hasAlgaeAtStartOnL2;

    ScoringSide(boolean hasAlgaeAtStartOnL2) {
      this.hasAlgaeAtStartOnL2 = hasAlgaeAtStartOnL2;
    }
  }
}
