package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.RunStateMachineCommand;
import frc.robot.autonomous.records.ScoringLocation.ReefSide;
import frc.robot.commands.PassToEndEffector;
import frc.robot.commands.PassToIntake;

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
    PassToEndEffector(false),
    PassToIntake(false);

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
    PassoffEndEffector,
    PassoffIntake,
    Stow,
    ScoreAlgae,
    ScoreCoralPrep,
    ScoreCoral,
    Climb,
    Down,
    Eject
  }

  public enum ScoringLevel {
    L1,
    L2,
    L3,
    L4
  }

  public enum TargetCoralPosition {
    Intake,
    EndEffector
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

  private static LoggerEntry.Bool logHighStowMode = logGroup.buildBoolean("HighStowMode");

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

  private static LoggerGroup logTransferCoral = logGroup.subgroup("TransferCoral");
  private static LoggerEntry.Bool logTransferToEndEffector =
      logTransferCoral.buildBoolean("TransferToEndEffector");
  private static LoggerEntry.Bool logTransferToIntake =
      logTransferCoral.buildBoolean("TransferToIntake");

  private static LoggerGroup logSubsystemsInUse = logGroup.subgroup("SubsystemsInUse");
  private static LoggerEntry.Bool logIntakeInUse = logSubsystemsInUse.buildBoolean("IntakeInUse");
  private static LoggerEntry.Bool logMechInUse = logSubsystemsInUse.buildBoolean("MechInUse");
  private static LoggerEntry.Bool logEndEffectorInUse =
      logSubsystemsInUse.buildBoolean("EndEffectorInUse");

  private static LoggerGroup logSubsystemsInTargetStates =
      logGroup.subgroup("SubsystemsInTargetStates");
  private static LoggerEntry.Bool logIntakeInTargetState =
      logSubsystemsInTargetStates.buildBoolean("IntakeInTargetState");
  private static LoggerEntry.Bool logMechInTargetState =
      logSubsystemsInTargetStates.buildBoolean("MechInTargetState");

  public static boolean clearingAlgae;

  public static ScoringLevel scoringLevel = ScoringLevel.L4;

  public static EndEffectorDesiredAction endEffectorDesiredAction = EndEffectorDesiredAction.Idle;
  public static double endEffectorOverrideVelocity = Double.NaN;

  public static MechState mechState = MechState.Idle;

  public static IntakeState intakeState = IntakeState.Idle;

  public static boolean algaeInRobot;

  public static boolean coralInRobot;

  public static boolean coralInEndEffector;

  public static boolean coralInEndEffectorScoringSide;
  public static boolean coralInEndEffectorNonScoringSide;

  public static boolean coralInIntake;

  public static boolean highStowMode = true;

  public static ReefSide targetReefSide = ReefSide.CA;

  public static Trigger triggerForCoralInRobot = new Trigger(() -> coralInRobot);
  public static Trigger triggerForCoralInEndEffector =
      new Trigger(() -> coralInEndEffector).debounce(0.5);
  public static Trigger triggerForCoralInIntake =
      new Trigger(() -> coralInIntake)
          .debounce(
              0.5); // debounce to allow coarl to get fully in intake before ending intake command

  public static TargetCoralPosition targetCoralPosition = TargetCoralPosition.EndEffector;

  private static boolean transferToEndEffector;
  private static boolean transferToIntake;

  public static boolean endEffectorInUse;
  public static boolean mechanismInUse;
  public static boolean intakeInUse;

  public static boolean intakeInTargetState;
  public static boolean mechInTargetState;

  public static Command passToEndEffector =
      new RunStateMachineCommand(() -> new PassToEndEffector());
  public static Command passToIntake = new RunStateMachineCommand(() -> new PassToIntake());

  public static void changeEndEffectorIfNotAligning(EndEffectorDesiredAction action) {
    if (!endEffectorDesiredAction.alignmentActive) {
      endEffectorDesiredAction = action;
    }
  }

  public static void periodic() {

    targetCoralPosition =
        scoringLevel == ScoringLevel.L1
            ? TargetCoralPosition.Intake
            : TargetCoralPosition.EndEffector;

    transferToEndEffector =
        targetCoralPosition == TargetCoralPosition.EndEffector
            && !coralInEndEffector
            && coralInIntake;
    transferToIntake =
        targetCoralPosition == TargetCoralPosition.Intake && !coralInIntake && coralInEndEffector;

    logTransferToEndEffector.info(transferToEndEffector);
    logTransferToIntake.info(transferToIntake);

    logIntakeInUse.info(intakeInUse);
    logMechInUse.info(mechanismInUse);
    logEndEffectorInUse.info(endEffectorInUse);

    logIntakeInTargetState.info(intakeInTargetState);
    logMechInTargetState.info(mechInTargetState);

    if (mechState == MechState.Override) {
      mechanismInUse = true;
      passToEndEffector.cancel();
      passToIntake.cancel();
    }

    if (intakeState == IntakeState.Override) {
      intakeInUse = true;
      passToEndEffector.cancel();
      passToIntake.cancel();
    }

    if (!mechanismInUse && !endEffectorInUse && !intakeInUse) {
      if (transferToEndEffector && !passToEndEffector.isScheduled()) {
        passToEndEffector.schedule();
      }

      if (transferToIntake && !passToIntake.isScheduled()) {
        passToIntake.schedule();
      }
    } else {
      passToEndEffector.cancel();
      passToIntake.cancel();
    }

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
    logHighStowMode.info(highStowMode);

    mechanismInUse = false;
    intakeInUse = false;
    endEffectorInUse = false;
  }

  public static void setMechState(MechState state) {
    mechState = state;
    mechanismInUse = true;
  }

  public static void setIntakeState(IntakeState state) {
    intakeState = state;
    intakeInUse = true;
  }

  public static void setEndEffectorState(EndEffectorDesiredAction state) {
    endEffectorDesiredAction = state;
    endEffectorInUse = true;
  }
}
