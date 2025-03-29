package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.robot.RobotStates;
import frc.robot.RobotStates.EndEffectorDesiredAction;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.MechState;
import frc.robot.subsystems.intake.Intake;

public class PassToEndEffector extends StateMachine {

  private final Intake intake;
  private final RobotStates states;

  private final Trigger endTrigger;
  private final Trigger lostCoral;

  private final LoggerGroup group = LoggerGroup.build("PassToEndEffector");
  private final LoggerEntry.Bool logPivotAtTargetAngle = group.buildBoolean("PivotAtTargetAngle");

  public PassToEndEffector(Intake intake, RobotStates states) {
    super("PassToEndEffector");

    this.intake = intake;
    this.states = states;

    endTrigger = new Trigger(() -> states.coralInEndEffectorScoringSide).debounce(0.3);
    lostCoral = new Trigger(() -> !states.coralInIntake).debounce(0.5);

    setInitialState(stateWithName("PrepToPassOff", () -> prepToPassOff()));
    setInterruptedState(stateWithName("End", () -> end()));
  }

  private StateHandler prepToPassOff() {
    states.mechState = MechState.PrepPassoffPosition;
    states.intakeState = IntakeState.PrepPassoff;
    states.endEffectorDesiredAction = EndEffectorDesiredAction.PassToEndEffector;
    if (lostCoral.getAsBoolean()) return stateWithName("End", () -> end());
    boolean pivotAtTargetAngle = intake.isPivotAtTargetAngle(intake.getPassOffPivotAngle());
    logPivotAtTargetAngle.info(pivotAtTargetAngle);
    return pivotAtTargetAngle && states.mechInTargetState
        ? stateWithName("PassOff", () -> passOff())
        : null;
  }

  private StateHandler passOff() {
    states.mechState = MechState.PassoffPosition;
    states.intakeState = IntakeState.PassoffEndEffector;
    states.endEffectorDesiredAction = EndEffectorDesiredAction.PassToEndEffector;
    if (lostCoral.getAsBoolean()) return stateWithName("End", () -> end());
    return endTrigger.getAsBoolean() ? stateWithName("End", () -> end()) : null;
  }

  private StateHandler end() {
    states.mechState = MechState.StowPosition;
    states.intakeState = IntakeState.Stow;
    states.endEffectorDesiredAction = EndEffectorDesiredAction.AlignCoral;
    return setDone();
  }
}
