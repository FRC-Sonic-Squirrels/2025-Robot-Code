package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.ExecutionTiming;
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
import frc.robot.FieldStates;
import frc.robot.RobotStates;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.MechState;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.helpers.ChoreoHelper.ChassisSpeedsWithPathEnd;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.autonomous.records.ScoringLocation.ReefSide;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.DriveToPosePathing;
import frc.robot.commands.endEffector.EndEffectorSetRPM;
import frc.robot.commands.mechanism.MechToPosition;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.LED.RobotState;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.mechanism.MechanismPositions;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ScoreCoral extends StateMachine {

  private final DrivetrainWrapper wrapper;
  private final Mechanism mech;
  private final Elevator elevator;
  private final Arm arm;
  private final LED led;
  private final RobotConfig config;
  private final Supplier<ScoreCoralObjective> objective;

  private final Optional<ScoringDirection> optionalScoringDirection;
  private ScoringDirection scoringDirection;
  private ScoringSide scoringSide;
  private final Optional<ScoringSide> optionalScoringSide;
  private ScoringSideWithPoseAndDirection scoringPoseSideAndDirection;
  private Pose2d algaeClearPose;
  private Pose2d scoringPose;
  private final Optional<ChoreoTrajectoryWithName> optionalPath;

  private final RobotStates states;

  private Command prepMechanismForScoring;

  private boolean usingDrivetrain = true;

  private final Consumer<Double> rumble;

  private boolean gamepieceMemory = false;

  private Trigger scoringTrigger;

  private final boolean confirmation;

  private ChoreoHelper choreoHelper;

  private boolean inPosition;

  private DriveToPosePathing alignToScore;

  private DriveToPose alignToScoreSimple;

  private boolean aligningWithCoralInWay;

  private static final TunableNumberGroup group = new TunableNumberGroup("ScoreCoral");
  private static final LoggedTunableNumber distToRaiseMech =
      group.build("DistToRaiseMechMeters", 0.05);
  private static final LoggedTunableNumber predictiveTime =
      group.build("PredictiveTimeSeconds", 0.3);
  private static final LoggedTunableNumber leftAdjustmentInches =
      group.build("A Adjustment Inches", 0.0);
  private static final LoggedTunableNumber rightAdjustmentInches =
      group.build("B Adjustment Inches", 0.0);
  private static final LoggedTunableNumber finalErrorMaxWait =
      group.build("FinalErrorMaxWait", 1.0);
  private static final LoggedTunableNumber finalHeadingError =
      group.build("FinalHeadingError", 0.5);
  private static final LoggedTunableNumber finalOffsetError = group.build("FinalOffsetError", 0.03);
  private static final LoggedTunableNumber finalTargetError = group.build("FinalTargetError", 0.03);
  private static final LoggedTunableNumber distToElevateCoral =
      group.build("DistToElevateCoral", 0.3);
  private static final LoggedTunableNumber velToElevateCoral =
      group.build("VelToElevateCoral", 1.5);
  private static final LoggedTunableNumber L2PrepPositionDistInches =
      group.build("L2PrepPositionDistInches", 4);

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
      Mechanism mech,
      Elevator elevator,
      Arm arm,
      LED led,
      Consumer<Double> rumble,
      RobotConfig config,
      Supplier<ScoreCoralObjective> objective,
      RobotStates robotStates) {
    this(
        wrapper,
        mech,
        elevator,
        arm,
        led,
        Optional.empty(),
        Optional.empty(),
        rumble,
        config,
        false,
        objective,
        Optional.empty(),
        robotStates);
    gamepieceMemory = true;
  }

  private boolean preloadCode;

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Mechanism mech,
      Elevator elevator,
      Arm arm,
      LED led,
      ReefSide side,
      Consumer<Double> rumble,
      RobotConfig config,
      Supplier<ScoreCoralObjective> objective,
      Optional<ChoreoTrajectoryWithName> optionalTraj,
      boolean preloadCode,
      RobotStates robotStates) {
    this(
        wrapper,
        mech,
        elevator,
        arm,
        led,
        Optional.of(reefSideToScoringDirection(side)),
        Optional.of(reefSideToScoringSide(side)),
        rumble,
        config,
        false,
        objective,
        optionalTraj,
        robotStates);
    this.preloadCode = preloadCode;
  }

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Mechanism mech,
      Elevator elevator,
      Arm arm,
      LED led,
      ScoringDirection scoringDirection,
      Consumer<Double> rumble,
      RobotConfig config,
      Supplier<ScoreCoralObjective> objective,
      RobotStates robotStates) {
    this(
        wrapper,
        mech,
        elevator,
        arm,
        led,
        Optional.of(scoringDirection),
        Optional.empty(),
        rumble,
        config,
        false,
        objective,
        Optional.empty(),
        robotStates);
  }

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Mechanism mech,
      Elevator elevator,
      Arm arm,
      LED led,
      Optional<ScoringDirection> scoringDirection,
      Optional<ScoringSide> side,
      Consumer<Double> rumble,
      RobotConfig config,
      boolean driverConfirmation,
      Supplier<ScoreCoralObjective> objective,
      Optional<ChoreoTrajectoryWithName> optionalPresetPath,
      RobotStates robotStates) {
    super("ScoreCoral");

    this.wrapper = wrapper;
    this.mech = mech;
    this.elevator = elevator;
    this.arm = arm;
    this.led = led;
    this.config = config;
    this.objective = objective;
    this.optionalPath = optionalPresetPath;
    this.states = robotStates;

    this.optionalScoringDirection = scoringDirection;
    this.rumble = rumble;
    this.optionalScoringSide = side;
    confirmation = driverConfirmation;

    setInterruptedState(stateWithName("End", () -> end(true)));
    setInitialState(stateWithName("ChooseAction", () -> chooseAction()));
  }

  private StateHandler chooseAction() {
    if (states.scoringLevel == ScoringLevel.L1
        && !(objective.get() == ScoreCoralObjective.JustClear))
      return stateWithName("ScoreL1", L1Position());

    Pose2d robotPose = wrapper.getReefPoseEstimatorPose(true);

    scoringPoseSideAndDirection =
        optionalScoringSide.isEmpty()
            ? getOptimalScoringSide(
                robotPose,
                wrapper.getFieldRelativeVelocities().getTranslation(),
                Units.Inches.of(0))
            : getScoringSide(optionalScoringSide.get());

    scoringPose = scoringPoseSideAndDirection.pose();

    scoringSide = scoringPoseSideAndDirection.side();

    scoringDirection = scoringPoseSideAndDirection.direction();

    log_scoringSide.info(scoringSide);
    log_scoringDirection.info(scoringDirection);
    log_scoringPose.info(scoringPose);
    log_algaeClearPose.info(scoringPoseSideAndDirection.pose());

    led.setBaseRobotState(BaseRobotState.SCORING_ALIGNMENT);

    if (objective.get() == ScoreCoralObjective.JustClear)
      return stateWithName("PrepForAlgaeAlignment", () -> prepForAlgaeAlignment());

    return states.scoringLevel == ScoringLevel.L2
        ? stateWithName("PrepL2Alignment", () -> prepL2Alignment())
        : stateWithName("PrepForScoringAlignment", prepForScoringAlignment());
  }

  // SCORING STATES

  private StateHandler L1Position() {
    states.intakeState = IntakeState.ScoreCoralPrep;
    states.mechState = MechState.AvoidIntake;
    return setDone();
  }

  private StateHandler prepL2Alignment() {
    alignToScore =
        new DriveToPosePathing(
                wrapper,
                config,
                () -> wrapper.getReefPoseEstimatorPose(true),
                () ->
                    getOptimalScoringSide(
                            wrapper.getReefPoseEstimatorPose(false),
                            wrapper.getFieldRelativeVelocities().getTranslation(),
                            Units.Inches.of(L2PrepPositionDistInches.get()))
                        .pose(),
                states.scoringLevel)
            .setFinalErrorMaxWait(finalErrorMaxWait.get())
            .setFinalOffsetError(finalOffsetError.get())
            .setFinalHeadingError(finalHeadingError.get())
            .setFinalTargetError(finalTargetError.get());
    spawnCommand(alignToScore, (c) -> null);
    return stateWithName("L2Alignment", () -> L2Alignment());
  }

  private StateHandler L2Alignment() {
    if (!states.coralInPassOff()) {
      states.mechState = MechState.ReefPosition;
      if (states.mechInTargetState)
        return stateWithName("PrepForScoringAlignment", prepForScoringAlignment());
    }
    return null;
  }

  private StateHandler prepForScoringAlignment() {
    if (!(states.coralInEndEffectorScoringSide
            || states.coralInEndEffectorNonScoringSide
            || states.coralInIntake
            || preloadCode)
        || !scorableLevel()) {
      return states.clearingAlgae && !gamepieceMemory
          ? stateWithName("PrepForAlgaeAlignment", () -> prepForAlgaeAlignment())
          : stateWithName("ScoreFailure", () -> scoreFailure());
    }

    states.targetReefSide = scoringSideAndDirectionToReefSide(scoringSide, scoringDirection);

    prepMechanismForScoring =
        spawnCommand(
            Commands.waitUntil(() -> !states.coralInPassOff())
                .andThen(
                    new MechToPosition(mech, MechState.StowPosition, states)
                        .alongWith(
                            Commands.waitUntil(
                                    () ->
                                        !(states.scoringLevel == ScoringLevel.L4)
                                            || (wrapper.getLinearVel() < velToElevateCoral.get()
                                                && GeometryUtil.getDist(
                                                        wrapper.getReefPoseEstimatorPose(true),
                                                        scoringPose)
                                                    < distToElevateCoral.get()))
                                .andThen(new MechToPosition(mech, MechState.ReefPosition, states))))
                .withName("MechScoreCoral"),
            (command) -> null);

    scoringTrigger =
        new Trigger(
                () ->
                    arm.isAtTargetAngle(
                        MechanismPositions.reefPosition(states.scoringLevel).armAngle(),
                        Rotation2d.fromDegrees(1.5)))
            .debounce(0);

    return optionalPath.isPresent()
        ? stateWithName("InitFollowPath", () -> initFollowPath())
        : stateWithName("InitFollowGeneratedPath", () -> InitFollowGeneratedPath());
  }

  private StateHandler InitFollowGeneratedPath() {
    alignToScore =
        new DriveToPosePathing(
                wrapper,
                config,
                () -> wrapper.getReefPoseEstimatorPose(true),
                () -> scoringPose,
                states.scoringLevel)
            .setFinalErrorMaxWait(finalErrorMaxWait.get())
            .setFinalOffsetError(finalOffsetError.get())
            .setFinalHeadingError(finalHeadingError.get())
            .setFinalTargetError(finalTargetError.get());
    spawnCommand(alignToScore.andThen(() -> inPosition = true), (c) -> null);
    return stateWithName("FollowGeneratedPath", () -> followGeneratedPath(false));
  }

  private StateHandler followGeneratedPath(boolean checkAlignment) {
    if (checkAlignment
        && alignToScore.pathStalling()
        && GeometryUtil.getDist(wrapper.getReefPoseEstimatorPose(false), scoringPose) < 0.5) {
      alignToScore.cancel();
      return stateWithName("PrepAlignWithCoralInTheWay", () -> prepAlignWithCoralInWay());
    }
    if (inPosition) {
      return stateWithName("Score", () -> score());
    }
    return null;
  }

  private StateHandler prepAlignWithCoralInWay() {
    alignToScore.cancel();
    Pose2d newScoringPose =
        getClosestScoringSide(
                wrapper.getReefPoseEstimatorPose(false).getTranslation(),
                Constants.FieldConstants.Gamepieces.CORAL_OUTER_DIAMETER)
            .pose();
    log_scoringPose.info(newScoringPose);
    alignToScoreSimple =
        new DriveToPose(
            wrapper, () -> newScoringPose, () -> wrapper.getReefPoseEstimatorPose(true));
    prepMechanismForScoring.cancel();
    if (!states.coralInPassOff()) states.mechState = MechState.ReefPositionCoralInWay;
    aligningWithCoralInWay = true;
    spawnCommand(alignToScoreSimple.andThen(() -> inPosition = true), (c) -> null);
    return stateWithName("FollowGeneratedPath", () -> followGeneratedPath(false));
  }

  private StateHandler initFollowPath() {
    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            wrapper.getReefPoseEstimatorPose(true),
            optionalPath.get(),
            config.getDriveBaseRadius() / 2,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());
    choreoHelper.setFinalErrorMaxWait(finalErrorMaxWait.get());
    choreoHelper.setFinalHeadingError(finalHeadingError.get());
    choreoHelper.setFinalOffsetError(finalOffsetError.get());
    choreoHelper.setFinalTargetError(finalTargetError.get());
    return stateWithName("FollowPath", () -> followPath());
  }

  private static final ExecutionTiming timingFollowPath = new ExecutionTiming("FollowPath");

  private StateHandler followPath() {
    ChassisSpeedsWithPathEnd result;
    try (var ignored = timingFollowPath.start()) {

      result =
          choreoHelper.calculateChassisSpeeds(
              wrapper.getReefPoseEstimatorPose(true), timeFromStart());
    
              wrapper.setVelocityOverride(result.chassisSpeeds());
    }
      inPosition = result.atEndOfPath();
    
       wrapper.setVelocityOverride(result.chassisSpeeds());
      if (!inPosition && !preloadCode) return null;
      wrapper.resetVelocityOverride();
      return stateWithName("Score", () -> score());
    
  }

  private StateHandler score() {

    if (scoringTrigger.getAsBoolean() && !confirmation) {
      states.endEffectorDesiredAction = RobotStates.EndEffectorDesiredAction.ScoreFastForward;
    }

    if (preloadCode) return stateWithName("End", () -> end(false));

    if (states.coralInEndEffector && !confirmation) return null;

    if (confirmation && prepMechanismForScoring.isScheduled()) return null;

    spawnCommand(new ControllerRumbleForTime(rumble, 0.25, 0.3), (c) -> null);

    led.setRobotState(RobotState.SCORE_SUCCESS);

    return objective.get() == ScoreCoralObjective.ScoreAndClear
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

    Command clearAlgae1Position =
        high
            ? new MechToPosition(mech, MechState.ClearAlgaeHighPosition, states)
            : new MechToPosition(mech, MechState.ClearAlgaeLowPosition, states);

    spawnCommand(clearAlgae1Position, (command) -> null);

    algaeClearPose = getClosestAlgaeClearingSide(scoringPose).pose();

    DriveToPose driveToAlgaePose =
        new DriveToPose(
            wrapper, () -> algaeClearPose, () -> wrapper.getReefPoseEstimatorPose(true));

    return suspendForCommand(
        new EndEffectorSetRPM(-6000, states)
            .finallyDo(() -> FieldStates.removeAlgaeFromScoringSide(scoringSide))
            .alongWith(driveToAlgaePose),
        (command) -> stateWithName("End", () -> end(false)));
  }

  private StateHandler scoreFailure() {
    spawnCommand(new ControllerRumbleForTime(rumble, 0.25, 0.3), (c) -> null);
    led.setRobotState(RobotState.SCORE_FAILURE);
    System.out.println("scoreFailure");
    return stateWithName("End", () -> end(true));
  }

  private StateHandler end(boolean interrupted) {
    led.setBaseRobotState(BaseRobotState.GAMEPIECE_STATUS);
    usingDrivetrain = false;
    return stateWithName("Done", setDone());
  }

  private ScoringSideWithPoseAndDirection getScoringSide(ScoringSide side) {
    for (ScoringSideWithPoseAndDirection scoringLocation :
        getScoringLocations(Units.Inches.of(0))) {
      if (scoringLocation.side() == side) {
        return scoringLocation;
      }
    }
    return null;
  }

  private ScoringSideWithPoseAndDirection getOptimalScoringSide(
      Pose2d robotPose, Translation2d vel, Distance extraDist) {

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

    return getClosestScoringSide(newTranslation, extraDist);
  }

  private ScoringSideWithPoseAndDirection getClosestScoringSide(
      Translation2d robotTranslation, Distance extraDist) {
    ScoringSideWithPoseAndDirection bestTarget =
        new ScoringSideWithPoseAndDirection(
            new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, Rotation2d.kZero),
            ScoringSide.FAR_LEFT,
            ScoringDirection.LEFT);

    for (ScoringSideWithPoseAndDirection pose : getScoringLocations(extraDist)) {
      if (scoreableLocation(
              new ScoringLocation(
                  scoringSideAndDirectionToReefSide(pose.side(), pose.direction()),
                  states.scoringLevel))
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
    for (ScoringSideWithPose pose : Constants.FieldConstants.SCORING_SIDES(Units.Inches.of(0))) {
      if (GeometryUtil.getDist(robotPose, pose.pose())
          < GeometryUtil.getDist(robotPose, bestTarget.pose())) {
        bestTarget = pose;
      }
    }
    return bestTarget;
  }

  private ScoringSideWithPoseAndDirection[] getScoringLocations(Distance extraDist) {

    var sides = Constants.FieldConstants.SCORING_SIDES(extraDist);

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
              Constants.FieldConstants.REEF_BRANCH_OFFSET.in(Units.Meters)
                  + Units.Inches.of(
                          direction == ScoringDirection.LEFT
                              ? -leftAdjustmentInches.get()
                              : rightAdjustmentInches.get())
                      .in(Units.Meter),
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
                new ScoringLocation(location.side(), states.scoringLevel))
            || conflictsWithAlgae(location)));
  }

  private boolean scorableLevel() {
    for (ScoringSideWithPoseAndDirection pose : getScoringLocations(Units.Inches.of(0))) {
      if (scoreableLocation(
          new ScoringLocation(
              scoringSideAndDirectionToReefSide(pose.side(), pose.direction()),
              states.scoringLevel))) return true;
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

  public enum ScoreCoralObjective {
    JustScore,
    JustClear,
    ScoreAndClear
  }
}
