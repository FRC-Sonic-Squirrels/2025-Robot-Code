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
import frc.robot.Constants.RobotMode;
import frc.robot.RobotStates;
import frc.robot.RobotStates.ScoringLevel;
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
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.Optional;
import java.util.function.Consumer;

public class ScoreCoral extends StateMachine {

  private final DrivetrainWrapper wrapper;
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endEffector;
  private final LED led;
  private final RobotConfig config;

  private final Optional<ScoringDirection> scoringDirection;
  private ScoringSide scoringSide;
  private final Optional<ScoringSide> optionalScoringSide;
  private ScoringSideWithPose scoringPoseAndSide;
  private Pose2d algaeClearPose;
  private Pose2d scoringPose;

  private Command prepMechanismForAlgae;
  private Command prepMechanismForScoring;

  private Command clearAlgae1Position;
  private Command clearAlgae2Position;

  private Command driveToPose;

  private final Consumer<Double> rumble;

  private static final TunableNumberGroup group = new TunableNumberGroup("ScoreCoral");
  private static final LoggedTunableNumber distToRaiseMech =
      group.build("DistToRaiseMechMeters", 1);
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
  }

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Elevator elevator,
      Arm arm,
      EndEffector endEffector,
      LED led,
      ReefSide side,
      Consumer<Double> rumble,
      RobotConfig config) {
    this(
        wrapper,
        elevator,
        arm,
        endEffector,
        led,
        Optional.of(reefSideToScoringDirection(side)),
        Optional.of(reefSideToScoringSide(side)),
        rumble,
        config);
  }

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Elevator elevator,
      Arm arm,
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
    this.endEffector = endEffector;
    this.led = led;
    this.config = config;

    this.scoringDirection = scoringDirection;
    this.rumble = rumble;
    this.optionalScoringSide = side;

    setInterruptedState(stateWithName("End", () -> end(true)));
    setInitialState(stateWithName("ChooseAlignment", () -> chooseAlignment()));
  }

  private StateHandler chooseAlignment() {
    Pose2d robotPose = wrapper.getReefPoseEstimatorPose(true);
    algaeClearPose = getClosestAlgaeClearingSide(robotPose).pose();
    scoringPoseAndSide =
        optionalScoringSide.isEmpty()
            ? getOptimalScoringSide(
                robotPose, wrapper.getFieldRelativeVelocities().getTranslation())
            : getScoringSide(optionalScoringSide.get());
    scoringPose = scoringPoseAndSide.pose();
    scoringSide = scoringPoseAndSide.side();

    log_scoringSide.info(scoringSide);
    if (scoringDirection.isPresent()) log_scoringDirection.info(scoringDirection.orElseThrow());
    log_scoringPose.info(scoringPose);
    log_algaeClearPose.info(scoringPoseAndSide.pose());

    driveToPose =
        new DriveToPose(
            wrapper,
            () -> algaeClearPose,
            () -> wrapper.getReefPoseEstimatorPose(true)); // change to path based alignment

    return algaeClearRequired()
        ? stateWithName("PrepForAlgaeAlignment", () -> prepForAlgaeAlignment())
        : stateWithName("PrepForScoringAlignment", () -> prepForScoringAlignment());
  }

  private StateHandler prepForAlgaeAlignment() {
    led.setBaseRobotState(BaseRobotState.ALGAE_ALIGNMENT);
    boolean high =
        scoringSide == ScoringSide.NEAR_MID
            || scoringSide == ScoringSide.FAR_LEFT
            || scoringSide == ScoringSide.FAR_RIGHT;

    clearAlgae1Position =
        high
            ? MechanismActions.clearAlgaeHigh1Position(elevator, arm)
            : MechanismActions.clearAlgaeLow1Position(elevator, arm);
    clearAlgae2Position =
        high
            ? MechanismActions.clearAlgaeHigh2Position(elevator, arm)
            : MechanismActions.clearAlgaeLow2Position(elevator, arm);

    prepMechanismForAlgae =
        spawnCommand(
            Commands.waitUntil(
                    () ->
                        GeometryUtil.getDist(wrapper.getReefPoseEstimatorPose(true), scoringPose)
                            < distToRaiseMech.get())
                .andThen(clearAlgae1Position),
            (command) -> null);

    return suspendForCommand(
        driveToPose,
        (command) ->
            suspendForCommand(
                Commands.waitUntil(prepMechanismForAlgae::isFinished).andThen(clearAlgae2Position),
                (c) ->
                    stateWithName(
                        "PrepForScoringAlignment",
                        () -> prepForScoringAlignment()))); // TODO: make algae clearing happen last
  }

  private StateHandler prepForScoringAlignment() {
    led.setBaseRobotState(BaseRobotState.SCORING_ALIGNMENT);
    if (!RobotStates.coralInEndEffector) {
      spawnCommand(new ControllerRumbleForTime(rumble, 0.25, 0.3), (c) -> null);
      led.setRobotState(RobotState.SCORE_FAILURE);
      return stateWithName("End", () -> end(true));
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

    if (!prepMechanismForScoring.isScheduled()) endEffector.setVelocity(scoringVelocityRPM.get());

    if (RobotMode.isSimBot()) {
      RobotStates.coralInEndEffectorNonScoringSide = false;
      RobotStates.coralInEndEffectorScoringSide = false;
    }

    if (RobotStates.coralInEndEffector) return null;

    spawnCommand(new ControllerRumbleForTime(rumble, 0.25, 0.3), (c) -> null);

    led.setRobotState(RobotState.SCORE_SUCCESS);

    return stateWithName("End", () -> end(false));
  }

  private StateHandler end(boolean interrupted) {
    led.setBaseRobotState(BaseRobotState.GAMEPIECE_STATUS);
    spawnCommand(
        MechanismActions.coralStationPosition(elevator, arm), // TODO: this may cause end
        // effector to hit the reef, potentially add intermediate position
        (command) -> setDone());
    endEffector.setPercentOut(0);
    return stateWithName("ResetMechanismToCoralPosition", () -> waitState());
  }

  private StateHandler waitState() {
    return null;
  }

  private boolean algaeClearRequired() {
    // TODO: potentially store locations where algae has been cleared, then return false if algae is
    // already cleared. May want to add manual override for this in case algae clear fails.
    // L1 and L4 are clear of algae
    if (RobotStates.scoringLevel == ScoringLevel.L1
        || RobotStates.scoringLevel == ScoringLevel.L4) {
      return false;
    }

    ScoringSide currentScoringSide = scoringSide;

    // There are particular sides of L2 that are free
    if (RobotStates.scoringLevel == ScoringLevel.L2
        && currentScoringSide != ScoringSide.NEAR_LEFT
        && currentScoringSide != ScoringSide.NEAR_RIGHT
        && currentScoringSide != ScoringSide.FAR_MID) {
      return false;
    }

    return RobotStates.clearingAlgae;
  }

  private ScoringSideWithPose getScoringSide(ScoringSide side) {
    for (ScoringSideWithPose scoringLocation : getScoringLocations()) {
      if (scoringLocation.side() == side) {
        return scoringLocation;
      }
    }
    return null;
  }

  private ScoringSideWithPose getOptimalScoringSide(Pose2d robotPose, Translation2d vel) {
    log_inputVel.info(vel);
    Translation2d flippedReef =
        AllianceFlipUtil.flipTranslationForAlliance(Constants.FieldConstants.BLUE_REEF_CENTER_POSE);
    Rotation2d headingToReef =
        GeometryUtil.getHeading(robotPose.getTranslation(), flippedReef).unaryMinus();
    log_headingToReef.info(headingToReef);
    // y' y * cos(t) + x * sin(t)
    // x' y * sin(t) + x * cos(t)
    // distToReef = dist(curr, reef)
    // distToReef + x' * time = newDistToReef
    // ravg = (distToReef + newDistToReef) / 2.0
    // deltaTheta = y' / ravg
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

  private ScoringSideWithPose getClosestScoringSide(Translation2d robotTranslation) {
    ScoringSideWithPose bestTarget =
        new ScoringSideWithPose(
            new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, Rotation2d.kZero), ScoringSide.FAR_LEFT);
    for (ScoringSideWithPose pose : getScoringLocations()) {
      if (GeometryUtil.getDist(robotTranslation, pose.pose().getTranslation())
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

  private ScoringSideWithPose[] getScoringLocations() {

    var sides = Constants.FieldConstants.SCORING_SIDES();

    var newSides = new ScoringSideWithPose[sides.length];

    for (int i = 0; i < sides.length * (scoringDirection.isPresent() ? 1 : 2); i++) {
      var scoringSidePose = sides[i * (scoringDirection.isPresent() ? 1 : 1 / 2)].pose();

      var scoringSide = sides[i * (scoringDirection.isPresent() ? 1 : 1 / 2)].side();

      Rotation2d objectiveScoringDirection;

      if (scoringDirection.isPresent()) {

        objectiveScoringDirection =
            scoringDirection.orElseThrow() == ScoringDirection.LEFT
                ? Rotation2d.kCW_90deg
                : Rotation2d.kCCW_90deg;

      } else {

        objectiveScoringDirection = i % 2 == 0 ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
      }

      Translation2d offset =
          new Translation2d(
              Constants.FieldConstants.REEF_BRANCH_OFFSET.in(Units.Meters),
              scoringSidePose.getRotation().plus(objectiveScoringDirection));
      Translation2d translation = scoringSidePose.getTranslation().plus(offset);
      newSides[i] =
          new ScoringSideWithPose(
              new Pose2d(translation, scoringSidePose.getRotation()), scoringSide);
    }

    return newSides;
  }

  private static ScoringDirection reefSideToScoringDirection(ReefSide side) {
    return (side.ordinal() % 2 == 0) ? ScoringDirection.LEFT : ScoringDirection.RIGHT;
  }

  private static ScoringSide reefSideToScoringSide(ReefSide side) {
    switch (side) {
      case CA:
      case CB:
        return ScoringSide.NEAR_MID;
      case CC:
      case CD:
        return ScoringSide.NEAR_RIGHT;
      case CE:
      case CF:
        return ScoringSide.FAR_RIGHT;
      case CG:
      case CH:
        return ScoringSide.FAR_MID;
      case CI:
      case CJ:
        return ScoringSide.FAR_LEFT;
      case CK:
      case CL:
        return ScoringSide.NEAR_LEFT;
      default:
        return null;
    }
  }

  public enum ScoringDirection {
    LEFT,
    RIGHT
  }

  public enum ScoringSide {
    NEAR_LEFT,
    NEAR_MID,
    NEAR_RIGHT,
    FAR_LEFT,
    FAR_MID,
    FAR_RIGHT
  }
}
