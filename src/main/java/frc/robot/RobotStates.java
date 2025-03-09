package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.autonomous.records.ScoringLocation.ReefSide;

public class RobotStates {
  public enum EndEffectorDesiredAction {
    Idle(false),
    CoralStationIntake(false),
    GroundIntake(false),
    AlignCoral(true),
    AlignCoralPhase2(true),
    AlignCoralPhase3(true),
    AlignCoralPhase4(true),
    AlignedCoral(true),
    ScoreFastForward(false),
    ScoreFastBackward(false),
    ScoreSlowForward(false),
    ScoreSlowBackward(false),
    PassToEndEffector(false);

    public final boolean alignmentActive;

    EndEffectorDesiredAction(boolean alignmentActive) {
      this.alignmentActive = alignmentActive;
    }
  }

  public enum MechState {
    Idle,
    Override,
    ReefPosition,
    ReefL1Position,
    ReefL2Position,
    ReefL3Position,
    ReefL4Position,
    ReefPrepPosition,
    CoralStationPosition,
    ClearAlgaeLowPosition,
    ClearAlgaeHighPosition,
    StowPosition,
    ClimbPosition,
    PrepPassoffPosition,
    PassoffPosition
  }

  public enum IntakeState {
    Idle,
    Override,
    IntakeCoral,
    IntakeAlgae,
    PrepPassoff,
    Passoff,
    Stow,
    ScoreAlgae,
    Climb
  }

  public enum ScoringLevel {
    L1,
    L2,
    L3,
    L4
  }

  private static LoggerGroup logGroup = LoggerGroup.build("RobotState");

  private static LoggerEntry.Bool logAlgaeClearingState = logGroup.buildBoolean("AlgaeClearing");
  private static LoggerEntry.Bool logGamepieceInRobotState =
      logGroup.buildBoolean("GamepieceInRobotState");
  private static LoggerEntry.Bool logGamepieceInEndEffectorState =
      logGroup.buildBoolean("GamepieceInEndEffectorState");
  private static LoggerEntry.Bool logGamepieceInEndEffectorScoringSideState =
      logGroup.buildBoolean("GamepieceInEndEffectorScoringSideState");
  private static LoggerEntry.Bool logGamepieceInEndEffectorNonScoringSideState =
      logGroup.buildBoolean("GamepieceInEndEffectorNonScoringSideState");
  private static LoggerEntry.Bool logGamepieceInIntakeState =
      logGroup.buildBoolean("GamepieceInIntakeState");
  private static LoggerEntry.Bool logCoralInIntakeState =
      logGroup.buildBoolean("CoralInIntakeState");

  private static LoggerEntry.EnumValue<EndEffectorDesiredAction> logEndEffectorDesiredAction =
      logGroup.buildEnum("EndEffectorDesiredAction");
  private static LoggerEntry.EnumValue<MechState> logMechState = logGroup.buildEnum("MechState");
  private static LoggerEntry.EnumValue<IntakeState> logIntakeState =
      logGroup.buildEnum("IntakeState");

  private static LoggerGroup logGroupLevels = logGroup.subgroup("Levels");
  private static LoggerEntry.EnumValue<ScoringLevel> logScoringLevelState =
      logGroupLevels.buildEnum("Level");
  private static LoggerEntry.Bool logL1State = logGroupLevels.buildBoolean("L1");
  private static LoggerEntry.Bool logL2State = logGroupLevels.buildBoolean("L2");
  private static LoggerEntry.Bool logL3State = logGroupLevels.buildBoolean("L3");
  private static LoggerEntry.Bool logL4State = logGroupLevels.buildBoolean("L4");

  public static boolean clearingAlgae;

  public static ScoringLevel scoringLevel = ScoringLevel.L4;

  public static EndEffectorDesiredAction endEffectorDesiredAction = EndEffectorDesiredAction.Idle;
  public static double endEffectorOverrideVelocity = Double.NaN;

  public static MechState mechState = MechState.Idle;

  public static IntakeState intakeState = IntakeState.Idle;

  public static boolean algaeInRobot;

  // TODO: update these coral values
  public static boolean coralInRobot;

  public static boolean coralInEndEffector;

  public static boolean coralInEndEffectorScoringSide;
  public static boolean coralInEndEffectorNonScoringSide;

  public static boolean coralInIntake;

  public static ReefSide targetReefSide = ReefSide.CA;

  public static Trigger triggerForCoralInRobot = new Trigger(() -> coralInRobot);
  public static Trigger triggerForCoralInEndEffector =
      new Trigger(() -> coralInEndEffector).debounce(0.5);
  public static Trigger triggerForCoralInIntake =
      new Trigger(() -> coralInIntake)
          .debounce(
              0.5); // debounce to allow coarl to get fully in intake before ending intake command

  public static void changeEndEffectorIfNotAligning(EndEffectorDesiredAction action) {
    if (!endEffectorDesiredAction.alignmentActive) {
      endEffectorDesiredAction = action;
    }
  }

  public static void periodic() {
    coralInRobot = coralInEndEffector || coralInIntake;

    logEndEffectorDesiredAction.info(endEffectorDesiredAction);
    logMechState.info(mechState);
    logIntakeState.info(intakeState);

    var level = scoringLevel;
    logScoringLevelState.info(level);
    logL1State.info(level == ScoringLevel.L1);
    logL2State.info(level == ScoringLevel.L2);
    logL3State.info(level == ScoringLevel.L3);
    logL4State.info(level == ScoringLevel.L4);
    logAlgaeClearingState.info(clearingAlgae);
    logGamepieceInRobotState.info(coralInRobot);
    logGamepieceInIntakeState.info(coralInIntake || algaeInRobot);
    logCoralInIntakeState.info(coralInIntake);
    logGamepieceInEndEffectorState.info(coralInEndEffector);
    logGamepieceInEndEffectorScoringSideState.info(coralInEndEffectorScoringSide);
    logGamepieceInEndEffectorNonScoringSideState.info(coralInEndEffectorNonScoringSide);
  }
}
