package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.commands.drive.SnapToReef;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.vision.Vision.TagOffset;
import java.util.function.Supplier;

public class ScoreCoral extends StateMachine {

  private final DrivetrainWrapper wrapper;

  private final ScoringDirection scoringDirection;
  private final Supplier<ScoringSide> scoringSideSupplier;
  private final Supplier<ScoringSideWithPose> scoringSidePoseSupplier;
  private final Supplier<Pose2d> scoringPose;
  private final Supplier<TagOffset> tagOffset;
  private final Supplier<Distance> distToWall;

  private Command snapToReef;

  private static final TunableNumberGroup group = new TunableNumberGroup("ScoreCoral");
  private static final LoggedTunableNumber usePoseForAlignment =
      group.build("usePoseForAlignment", 1);

  private static final LoggerGroup log_group = LoggerGroup.build("ScoreCoral");
  private static final LoggerEntry.EnumValue<ScoringSide> log_scoringSide =
      log_group.buildEnum("ScoringSide");
  private static final LoggerEntry.EnumValue<ScoringDirection> log_scoringDirection =
      log_group.buildEnum("ScoringDirection");
  private static final LoggerEntry.Struct<Pose2d> log_scoringPose =
      log_group.buildStruct(Pose2d.class, "ScoringPose");

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Supplier<TagOffset[]> tagOffsets,
      Supplier<Distance> distToWall,
      ScoringDirection scoringDirection) {
    super("ScoreCoral");

    this.wrapper = wrapper;

    this.scoringDirection = scoringDirection;
    tagOffset = () -> tagOffsets.get()[scoringDirection == ScoringDirection.LEFT ? 0 : 1];
    scoringSidePoseSupplier = () -> getClosestScoringSide(wrapper.getPoseEstimatorPose(true));
    scoringPose = () -> scoringSidePoseSupplier.get().pose();
    this.distToWall = distToWall;
    this.scoringSideSupplier =
        () ->
            usePoseForAlignment.get() == 1
                ? scoringSidePoseSupplier.get().side()
                : getScoringSide(tagOffset.get().tagID());

    setInterruptedState(stateWithName("EndState", () -> end(true)));
    setInitialState(stateWithName("PrepAlignment", () -> prepForAlignment()));
  }

  private StateHandler prepForAlignment() {
    spawnCommand(
        usePoseForAlignment.get() == 1
            ? new DriveToPose(wrapper, scoringPose, () -> wrapper.getPoseEstimatorPose(true))
            : new SnapToReef(tagOffset, wrapper, distToWall),
        (command) -> {
          return algaeClearRequired()
              ? stateWithName("ClearAlgae", () -> clearAlgae())
              : stateWithName("Score", () -> score());
        });

    return stateWithName("Align", () -> align());
  }

  private StateHandler align() {

    log_scoringSide.info(scoringSideSupplier.get());
    log_scoringDirection.info(scoringDirection);

    if (usePoseForAlignment.get() == 1) log_scoringPose.info(scoringPose.get());
    // TODO: prep mechanism
    return null;
  }

  private StateHandler clearAlgae() {
    // TODO: add clear algae logic
    return stateWithName("Score", () -> score());
  }

  private StateHandler score() {
    // TODO: add score logic, vibrate controller when note is released
    return () -> end(false);
  }

  private StateHandler end(boolean interrupted) {
    // TODO: reset mechanism to pickup position, turn off LEDs
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

  private ScoringSide getScoringSide(int tagID) {
    if (tagID == 6) {
      return ScoringSide.NEAR_LEFT;
    } else if (tagID == 7) {
      return ScoringSide.NEAR_MID;
    } else if (tagID == 8) {
      return ScoringSide.NEAR_RIGHT;
    } else if (tagID == 9) {
      return ScoringSide.FAR_RIGHT;
    } else if (tagID == 10) {
      return ScoringSide.FAR_MID;
    } else if (tagID == 11) {
      return ScoringSide.FAR_LEFT;
    } else if (tagID == 17) {
      return ScoringSide.NEAR_RIGHT;
    } else if (tagID == 18) {
      return ScoringSide.NEAR_MID;
    } else if (tagID == 19) {
      return ScoringSide.NEAR_LEFT;
    } else if (tagID == 20) {
      return ScoringSide.FAR_LEFT;
    } else if (tagID == 21) {
      return ScoringSide.FAR_MID;
    } else {
      return ScoringSide.FAR_RIGHT;
    }
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
