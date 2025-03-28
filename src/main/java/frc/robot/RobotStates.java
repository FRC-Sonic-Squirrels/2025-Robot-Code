package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.RunStateMachineCommand;
import frc.robot.autonomous.records.ScoringLocation.ReefSide;
import frc.robot.commands.PassToEndEffector;
import frc.robot.commands.PassToIntake;
import frc.robot.subsystems.intake.Intake;

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
    PassCoralToEndEffector(false),
    PassToIntake(false),
    HoldAlgae(false),
    ScoreAlgae(false);

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
    PrepPassoffCoralPosition,
    PrepPassoffAlgaePosition,
    PassOffCoralPosition,
    PassOffAlgaePosition,
    HoldAlgaePosition,
    AvoidIntake,
    Default,
    AutoPrep,
    AlgaeBargePosition
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
    ScoreAlgaePrep,
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

  private LoggerGroup logGroup = LoggerGroup.build("RobotState");

  private LoggerEntry.Bool logAlgaeClearingState = logGroup.buildBoolean("AlgaeClearing");
  private LoggerEntry.Bool logGamepieceInRobotState =
      logGroup.buildBoolean("GamepieceInRobotState");
  private LoggerEntry.Bool logGamepieceInEndEffectorState =
      logGroup.buildBoolean("GamepieceInEndEffectorState");
  private LoggerEntry.Bool logGamepieceInEndEffectorScoringSideState =
      logGroup.buildBoolean("GamepieceInEndEffectorScoringSideState");
  private LoggerEntry.Bool logGamepieceInEndEffectorNonScoringSideState =
      logGroup.buildBoolean("GamepieceInEndEffectorNonScoringSideState");
  private LoggerEntry.Bool logGamepieceInIntakeState =
      logGroup.buildBoolean("GamepieceInIntakeState");
  private LoggerEntry.Bool logCoralInIntakeState = logGroup.buildBoolean("CoralInIntakeState");

  private LoggerEntry.Bool logHighStowMode = logGroup.buildBoolean("HighStowMode");

  private LoggerEntry.EnumValue<EndEffectorDesiredAction> logEndEffectorDesiredAction =
      logGroup.buildEnum("EndEffectorDesiredAction");
  private LoggerEntry.EnumValue<MechState> logMechState = logGroup.buildEnum("MechState");
  private LoggerEntry.EnumValue<IntakeState> logIntakeState = logGroup.buildEnum("IntakeState");

  private LoggerGroup logGroupLevels = logGroup.subgroup("Levels");
  private LoggerEntry.EnumValue<ScoringLevel> logScoringLevelState =
      logGroupLevels.buildEnum("Level");
  private LoggerEntry.Bool logL1State = logGroupLevels.buildBoolean("L1");
  private LoggerEntry.Bool logL2State = logGroupLevels.buildBoolean("L2");
  private LoggerEntry.Bool logL3State = logGroupLevels.buildBoolean("L3");
  private LoggerEntry.Bool logL4State = logGroupLevels.buildBoolean("L4");

  private LoggerGroup logTransferCoral = logGroup.subgroup("TransferCoral");
  private LoggerEntry.Bool logTransferCoralToEndEffector =
      logTransferCoral.buildBoolean("TransferToEndEffector");
  private LoggerEntry.Bool logTransferCoralToIntake =
      logTransferCoral.buildBoolean("TransferToIntake");
  private LoggerEntry.Bool logTransferAlgaeToEndEffector =
      logTransferCoral.buildBoolean("TransferToEndEffector");
  private LoggerEntry.Bool logTransferAlgaeToIntake =
      logTransferCoral.buildBoolean("TransferToIntake");

  private LoggerGroup logSubsystemsInUse = logGroup.subgroup("SubsystemsInUse");
  private LoggerEntry.Bool logIntakeInUse = logSubsystemsInUse.buildBoolean("IntakeInUse");
  private LoggerEntry.Bool logMechInUse = logSubsystemsInUse.buildBoolean("MechInUse");
  private LoggerEntry.Bool logEndEffectorInUse =
      logSubsystemsInUse.buildBoolean("EndEffectorInUse");

  private LoggerGroup logSubsystemsInTargetStates = logGroup.subgroup("SubsystemsInTargetStates");
  private LoggerEntry.Bool logIntakeInTargetState =
      logSubsystemsInTargetStates.buildBoolean("IntakeInTargetState");
  private LoggerEntry.Bool logMechInTargetState =
      logSubsystemsInTargetStates.buildBoolean("MechInTargetState");

  public boolean clearingAlgae;

  public ScoringLevel scoringLevel = ScoringLevel.L4;

  public EndEffectorDesiredAction endEffectorDesiredAction = EndEffectorDesiredAction.Idle;
  public double endEffectorOverrideVelocity = Double.NaN;

  public MechState mechState = MechState.Idle;

  public IntakeState intakeState = IntakeState.Idle;

  public boolean algaeInRobot;
  public boolean algaeInEndEffector;
  public boolean algaeInIntake;

  public boolean coralInRobot;

  public boolean coralInEndEffector;

  public boolean coralInEndEffectorScoringSide;
  public boolean coralInEndEffectorNonScoringSide;

  public boolean coralInIntake;

  public boolean highStowMode = true;

  public ReefSide targetReefSide = ReefSide.CA;

  public Trigger triggerForCoralInRobot = new Trigger(() -> coralInRobot);
  public Trigger triggerForCoralInEndEffector = new Trigger(() -> coralInEndEffector).debounce(0.5);
  public Trigger triggerForCoralInIntake =
      new Trigger(() -> coralInIntake)
          .debounce(
              0.5); // debounce to allow coral to get fully in intake before ending intake command

  public Trigger instantTriggerForCoralInIntake = new Trigger(() -> coralInIntake);
  // command
  public TargetCoralPosition targetCoralPosition = TargetCoralPosition.EndEffector;

  private boolean transferCoralToEndEffector;
  private boolean transferCoralToIntake;

  private boolean transferAlgaeToEndEffector;
  private boolean transferAlgaeToIntake;

  public boolean endEffectorInUse;
  public boolean mechanismInUse;
  public boolean intakeInUse;

  public boolean intakeInTargetState;
  public boolean mechInTargetState;

  private Command passCoralToEndEffector = Commands.none();
  private Command passCoralToIntake = new RunStateMachineCommand(() -> new PassToIntake(this));

  private Command passAlgaeToEndEffector = Commands.none();
  private Command passAlgaeToIntake = new RunStateMachineCommand(() -> new PassToIntake(this));

  public RobotStates() {}

  public void setIntake(Intake intake) {
    passCoralToEndEffector =
        new RunStateMachineCommand(() -> new PassToEndEffector(intake, this, true));
    passAlgaeToEndEffector =
        new RunStateMachineCommand(() -> new PassToEndEffector(intake, this, false));
  }

  public void changeEndEffectorIfNotAligning(EndEffectorDesiredAction action) {
    if (!endEffectorDesiredAction.alignmentActive) {
      endEffectorDesiredAction = action;
    }
  }

  public void periodic() {

    targetCoralPosition =
        scoringLevel == ScoringLevel.L1
            ? TargetCoralPosition.Intake
            : TargetCoralPosition.EndEffector;

    transferCoralToEndEffector =
        targetCoralPosition == TargetCoralPosition.EndEffector
            && !coralInEndEffector
            && coralInIntake;

    transferAlgaeToEndEffector = algaeInIntake && !algaeInEndEffector;

    transferCoralToIntake =
        targetCoralPosition == TargetCoralPosition.Intake && !coralInIntake && coralInEndEffector;

    logTransferCoralToEndEffector.info(transferCoralToEndEffector);
    logTransferCoralToIntake.info(transferCoralToIntake);

    logTransferAlgaeToEndEffector.info(transferAlgaeToEndEffector);
    logTransferAlgaeToIntake.info(transferAlgaeToIntake);

    logIntakeInUse.info(intakeInUse);
    logMechInUse.info(mechanismInUse);
    logEndEffectorInUse.info(endEffectorInUse);

    logIntakeInTargetState.info(intakeInTargetState);
    logMechInTargetState.info(mechInTargetState);

    if (mechState == MechState.Override) {
      mechanismInUse = true;
    }

    if (intakeState == IntakeState.Override) {
      intakeInUse = true;
    }

    if (!mechanismInUse && !endEffectorInUse && !intakeInUse) {
      if (transferCoralToEndEffector && !passCoralToEndEffector.isScheduled()) {
        passCoralToEndEffector.schedule();
      }

      if (transferCoralToIntake && !passCoralToIntake.isScheduled()) {
        passCoralToIntake.schedule();
      }

      if (transferAlgaeToEndEffector && !passAlgaeToEndEffector.isScheduled()) {
        passAlgaeToEndEffector.schedule();
      }

      if (transferAlgaeToIntake && !passAlgaeToIntake.isScheduled()) {
        passAlgaeToIntake.schedule();
      }
    } else {
      passCoralToEndEffector.cancel();
      passCoralToIntake.cancel();

      passAlgaeToEndEffector.cancel();
      passAlgaeToIntake.cancel();
    }

    coralInRobot = coralInEndEffector || coralInIntake;

    algaeInRobot = algaeInEndEffector || algaeInIntake;

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

  public void setMechState(MechState state) {
    mechState = state;
    mechanismInUse = true;
  }

  public void setIntakeState(IntakeState state) {
    intakeState = state;
    intakeInUse = true;
  }

  public void setEndEffectorState(EndEffectorDesiredAction state) {
    endEffectorDesiredAction = state;
    endEffectorInUse = true;
  }

  public boolean coralInPassOff() {
    return passCoralToEndEffector.isScheduled() || passCoralToIntake.isScheduled();
  }
}
