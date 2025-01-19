package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ScoringSideWithPose;
import frc.robot.RobotStates;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;

public class ScoreCoral extends StateMachine {

  private final DrivetrainWrapper wrapper;
  private final Elevator elevator;
  private final Arm arm;

  private final ScoringDirection scoringDirection;
  private final Supplier<ScoringSide> scoringSideSupplier;
  private final Supplier<ScoringSideWithPose> scoringSidePoseSupplier;
  private final Supplier<Pose2d> scoringPose;

  private static final TunableNumberGroup group = new TunableNumberGroup("ScoreCoral");
  private static final LoggedTunableNumber distToRaiseMech =
      group.build("DistToRaiseMechMeters", 1);

  private static final LoggerGroup log_group = LoggerGroup.build("ScoreCoral");
  private static final LoggerEntry.EnumValue<ScoringSide> log_scoringSide =
      log_group.buildEnum("ScoringSide");
  private static final LoggerEntry.EnumValue<ScoringDirection> log_scoringDirection =
      log_group.buildEnum("ScoringDirection");
  private static final LoggerEntry.Struct<Pose2d> log_scoringPose =
      log_group.buildStruct(Pose2d.class, "ScoringPose");

  public ScoreCoral(
      DrivetrainWrapper wrapper, Elevator elevator, Arm arm, ScoringDirection scoringDirection) {
    super("ScoreCoral");

    this.wrapper = wrapper;
    this.elevator = elevator;
    this.arm = arm;

    this.scoringDirection = scoringDirection;
    scoringSidePoseSupplier = () -> getClosestScoringSide(wrapper.getPoseEstimatorPose(true));
    scoringPose = () -> scoringSidePoseSupplier.get().pose();
    this.scoringSideSupplier = () -> scoringSidePoseSupplier.get().side();

    setInterruptedState(stateWithName("EndState", () -> end(true)));
    setInitialState(stateWithName("PrepAlignment", () -> prepForAlignment()));
  }

  private StateHandler prepForAlignment() {
    if(!RobotStates.coralInEndEffector){
      return stateWithName("End", () -> end(true));
    }

    spawnCommand(
        new DriveToPose(wrapper, scoringPose, () -> wrapper.getPoseEstimatorPose(true)),
        (command) -> {
          return algaeClearRequired()
              ? stateWithName("ClearAlgae", () -> clearAlgae())
              : stateWithName("Score", () -> score());
        });

    spawnCommand(
        Commands.waitUntil(() -> GeometryUtil.getDist(wrapper.getPoseEstimatorPose(true), scoringPose.get()) < distToRaiseMech.get()).andThen(MechanismActions.reefPosition(elevator, arm, RobotStates.scoringLevel)), (command) -> null);

    return stateWithName("Align", () -> align());
  }

  private StateHandler align() {

    log_scoringSide.info(scoringSideSupplier.get());
    log_scoringDirection.info(scoringDirection);
    log_scoringPose.info(scoringPose.get());

    return null;
  }

  private StateHandler clearAlgae() {
    // TODO: add clear algae logic
    return stateWithName("Score", () -> score());
  }

  private StateHandler score() {
    // TODO: add score logic, vibrate controller when note is released
    return stateWithName("End", () -> end(false));
  }

  private StateHandler end(boolean interrupted) {
    // TODO: reset mechanism to pickup position, turn off LEDs
    spawnCommand(
        MechanismActions.stowPosition(elevator, arm),
        (command) -> null); // potentially change to coral station position
    return setDone();
  }

  private boolean algaeClearRequired() {
    // L1 and L4 are clear of algae
    if (RobotStates.scoringLevel == ScoringLevel.L1
        || RobotStates.scoringLevel == ScoringLevel.L4) {
      return false;
    }

    ScoringSide currentScoringSide = scoringSideSupplier.get();

    // There are particular sides of L2 that are free
    if (RobotStates.scoringLevel == ScoringLevel.L2
        && currentScoringSide != ScoringSide.NEAR_LEFT
        && currentScoringSide != ScoringSide.NEAR_RIGHT
        && currentScoringSide != ScoringSide.FAR_MID) {
      return false;
    }

    return RobotStates.clearingAglae;
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
                      scoringSide == ScoringSide.FAR_LEFT
                              || scoringSide == ScoringSide.FAR_MID
                              || scoringSide == ScoringSide.FAR_RIGHT
                          ? objectiveScoringDirection
                          : objectiveScoringDirection.unaryMinus()));
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
