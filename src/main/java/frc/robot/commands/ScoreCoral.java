package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team2930.lib.controller_rumble.ControllerRumbleForTime;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ScoringSideWithPose;
import frc.robot.RobotStates;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.LED.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Consumer;

public class ScoreCoral extends StateMachine {

  private final DrivetrainWrapper wrapper;
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endEffector;
  private final LED led;

  private final ScoringDirection scoringDirection;
  private final ScoringSide scoringSide;
  private final ScoringSideWithPose scoringPoseAndSide;
  private final Pose2d algaeClearPose;
  private final Pose2d scoringPose;

  private Command prepMechanismForAlgae;
  private Command prepMechanismForScoring;

  private Command clearAlgae1Position;
  private Command clearAlgae2Position;

  private final Command driveToPose;

  private final Consumer<Double> rumble;

  private static final TunableNumberGroup group = new TunableNumberGroup("ScoreCoral");
  private static final LoggedTunableNumber distToRaiseMech =
      group.build("DistToRaiseMechMeters", 1);
  private static final LoggedTunableNumber scoringVelocityRPM =
      group.build("ScoringVelocityRPM", 1000);

  private static final LoggerGroup log_group = LoggerGroup.build("ScoreCoral");
  private static final LoggerEntry.EnumValue<ScoringSide> log_scoringSide =
      log_group.buildEnum("ScoringSide");
  private static final LoggerEntry.EnumValue<ScoringDirection> log_scoringDirection =
      log_group.buildEnum("ScoringDirection");
  private static final LoggerEntry.Struct<Pose2d> log_scoringPose =
      log_group.buildStruct(Pose2d.class, "ScoringPose");
  private static final LoggerEntry.Struct<Pose2d> log_algaeClearPose =
      log_group.buildStruct(Pose2d.class, "ScoringPose");

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Elevator elevator,
      Arm arm,
      EndEffector endEffector,
      LED led,
      ScoringDirection scoringDirection,
      Consumer<Double> rumble) {
    super("ScoreCoral");

    this.wrapper = wrapper;
    this.elevator = elevator;
    this.arm = arm;
    this.endEffector = endEffector;
    this.led = led;

    this.scoringDirection = scoringDirection;
    this.rumble = rumble;

    Pose2d robotPose = wrapper.getPoseEstimatorPose(true);
    algaeClearPose = getClosestAlgaeClearingSide(robotPose).pose();
    scoringPoseAndSide = getClosestScoringSide(robotPose);
    scoringPose = scoringPoseAndSide.pose();
    this.scoringSide = scoringPoseAndSide.side();
    driveToPose =
        new DriveToPose(wrapper, () -> algaeClearPose, () -> wrapper.getPoseEstimatorPose(true));

    log_scoringSide.info(scoringSide);
    log_scoringDirection.info(scoringDirection);
    log_scoringPose.info(scoringPose);
    log_algaeClearPose.info(scoringPoseAndSide.pose());

    setInterruptedState(stateWithName("End", () -> end(true)));
    setInitialState(stateWithName("ChooseAlignment", () -> chooseAlignment()));
  }

  private StateHandler chooseAlignment() {
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

    spawnCommand(
        driveToPose, (command) -> stateWithName("ClearAlgae", () -> initializeClearAlgae()));

    prepMechanismForAlgae =
        spawnCommand(
            Commands.waitUntil(
                    () ->
                        GeometryUtil.getDist(wrapper.getPoseEstimatorPose(true), scoringPose)
                            < distToRaiseMech.get())
                .andThen(clearAlgae1Position),
            (command) -> null);

    return stateWithName("AlignForAlgae", () -> waitState());
  }

  private StateHandler prepForScoringAlignment() {
    led.setBaseRobotState(BaseRobotState.SCORING_ALIGNMENT);
    if (!RobotStates.coralInEndEffector) {
      spawnCommand(new ControllerRumbleForTime(rumble, 0.25, 0.3), (c) -> null);
      led.setRobotState(RobotState.SCORE_FAILURE);
      return stateWithName("End", () -> end(true));
    }

    spawnCommand(
        new DriveToPose(wrapper, () -> scoringPose, () -> wrapper.getPoseEstimatorPose(true)),
        (command) -> {
          return stateWithName("Score", () -> score());
        });

    prepMechanismForScoring =
        spawnCommand(
            Commands.waitUntil(
                    () ->
                        GeometryUtil.getDist(wrapper.getPoseEstimatorPose(true), scoringPose)
                            < distToRaiseMech.get())
                .andThen(MechanismActions.reefPosition(elevator, arm, RobotStates.scoringLevel)),
            (command) -> null);

    return stateWithName("AlignForScoring", () -> waitState());
  }

  private StateHandler initializeClearAlgae() {

    spawnCommand(
        Commands.waitUntil(prepMechanismForAlgae::isFinished).andThen(clearAlgae2Position),
        (command) -> stateWithName("PrepForScoringAlignment", () -> prepForScoringAlignment()));

    return stateWithName("ClearAlgae", () -> waitState());
  }

  private StateHandler score() {

    if (!prepMechanismForScoring.isScheduled()) endEffector.setVelocity(scoringVelocityRPM.get());

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

  private ScoringSideWithPose getClosestScoringSide(Pose2d robotPose) {
    ScoringSideWithPose bestTarget =
        new ScoringSideWithPose(
            new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, Rotation2d.kZero), ScoringSide.FAR_LEFT);
    for (ScoringSideWithPose pose : getScoringLocations()) {
      if (GeometryUtil.getDist(robotPose, pose.pose())
          < GeometryUtil.getDist(robotPose, bestTarget.pose())) {
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

    for (int i = 0; i < sides.length; i++) {
      var scoringSidePose = sides[i].pose();

      var scoringSide = sides[i].side();

      var objectiveScoringDirection =
          scoringDirection == ScoringDirection.LEFT ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;

      Translation2d offset =
          new Translation2d(
              Constants.FieldConstants.REEF_BRANCH_OFFSET.in(Units.Meters),
              scoringSidePose
                  .getRotation()
                  .plus(
                      // scoringSide == ScoringSide.FAR_LEFT
                      //         || scoringSide == ScoringSide.FAR_MID
                      //         || scoringSide == ScoringSide.FAR_RIGHT
                      //     ?
                      objectiveScoringDirection
                      // : objectiveScoringDirection.unaryMinus()
                      ));
      Translation2d translation = scoringSidePose.getTranslation().plus(offset);
      newSides[i] =
          new ScoringSideWithPose(
              new Pose2d(translation, scoringSidePose.getRotation()), scoringSide);
    }

    return newSides;
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
