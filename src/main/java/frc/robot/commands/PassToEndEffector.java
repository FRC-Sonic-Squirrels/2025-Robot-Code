package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.RobotStates;
import frc.robot.RobotStates.EndEffectorDesiredAction;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.MechState;
import frc.robot.subsystems.intake.Intake;

public class PassToEndEffector extends StateMachine {

  private final Intake intake;
  private final RobotStates states;
  private final boolean coral;

  private final Trigger endTrigger;
  private final Trigger lostGamepiece;

  private final LoggerGroup group = LoggerGroup.build("PassToEndEffector");
  private final LoggerEntry.Bool logPivotAtTargetAngle = group.buildBoolean("PivotAtTargetAngle");

  private final TunableNumberGroup tunableGroup = new TunableNumberGroup("PassToEndEffector");
  private final LoggedTunableNumber algaeWait = tunableGroup.build("AlgaeWait", 1);

  public PassToEndEffector(Intake intake, RobotStates states, boolean coral) {
    super("PassToEndEffector");

    this.intake = intake;
    this.states = states;
    this.coral = coral;

    endTrigger =
        new Trigger(
                () -> (coral ? states.coralInEndEffectorScoringSide : states.algaeInEndEffector))
            .debounce(0.1);
    lostGamepiece =
        new Trigger(() -> !(coral ? states.coralInIntake : states.algaeInIntake)).debounce(0.5);

    setInitialState(stateWithName("PrepToPassOff", () -> prepToPassOff()));
    setInterruptedState(stateWithName("End", () -> end()));
  }

  private StateHandler prepToPassOff() {
    states.coralExpectedInEndEffector = coral;
    states.mechState =
        coral ? MechState.PrepPassoffCoralPosition : MechState.PrepPassoffAlgaePosition;
    states.intakeState = IntakeState.PrepPassoff;
    states.endEffectorDesiredAction = EndEffectorDesiredAction.PassCoralToEndEffector;
    if (lostGamepiece.getAsBoolean()) return stateWithName("End", () -> end());
    boolean pivotAtTargetAngle = intake.isPivotAtTargetAngle(intake.getPassOffPivotAngle());
    logPivotAtTargetAngle.info(pivotAtTargetAngle);
    return intake.isPivotAtTargetAngle(intake.getPassOffPivotAngle())
            && (!coral || states.mechInTargetState)
        ? (coral
            ? stateWithName("PassOff", () -> passOff())
            : stateWithName("ExtraWaitForAlgae", () -> extraWaitForAlgae()))
        : null;
  }

  private StateHandler extraWaitForAlgae() {
    states.intakeState = IntakeState.PassoffEndEffector;
    return timeFromStartOfState() > algaeWait.get()
        ? stateWithName("PassOff", () -> passOff())
        : null;
  }

  private StateHandler passOff() {
    states.mechState = coral ? MechState.PassOffCoralPosition : MechState.PassOffAlgaePosition;
    states.intakeState = IntakeState.PassoffEndEffector;
    states.endEffectorDesiredAction =
        coral
            ? EndEffectorDesiredAction.PassCoralToEndEffector
            : EndEffectorDesiredAction.HoldAlgae;

    if (lostGamepiece.getAsBoolean()) return stateWithName("End", () -> end());
    return endTrigger.getAsBoolean() ? stateWithName("End", () -> end()) : null;
  }

  private StateHandler end() {
    states.mechState = coral ? MechState.StowPosition : MechState.HoldAlgaePosition;
    states.intakeState = IntakeState.Stow;
    states.endEffectorDesiredAction =
        coral ? EndEffectorDesiredAction.AlignCoral : EndEffectorDesiredAction.HoldAlgae;
    return setDone();
  }
}
