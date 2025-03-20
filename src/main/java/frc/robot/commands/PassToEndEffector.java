package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.StateMachine;
import frc.robot.RobotStates;
import frc.robot.RobotStates.EndEffectorDesiredAction;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.MechState;

public class PassToEndEffector extends StateMachine {

  private final Trigger endTrigger =
      new Trigger(() -> RobotStates.coralInEndEffectorScoringSide).debounce(0.3);
  private final Trigger lostCoral = new Trigger(() -> !RobotStates.coralInIntake).debounce(0.5);

  public PassToEndEffector() {
    super("PassToEndEffector");

    setInitialState(stateWithName("PrepToPassOff", () -> prepToPassOff()));
    setInterruptedState(stateWithName("End", () -> end()));
  }

  private StateHandler prepToPassOff() {
    RobotStates.mechState = MechState.PrepPassoffPosition;
    RobotStates.intakeState = IntakeState.PrepPassoff;
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.PassToEndEffector;
    if (lostCoral.getAsBoolean()) return stateWithName("End", () -> end());
    return RobotStates.intakeInTargetState && RobotStates.mechInTargetState
        ? stateWithName("PassOff", () -> passOff())
        : null;
  }

  private StateHandler passOff() {
    RobotStates.mechState = MechState.PassoffPosition;
    RobotStates.intakeState = IntakeState.PassoffEndEffector;
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.PassToEndEffector;
    if (lostCoral.getAsBoolean()) return stateWithName("End", () -> end());
    return endTrigger.getAsBoolean() ? stateWithName("End", () -> end()) : null;
  }

  private StateHandler end() {
    RobotStates.mechState = MechState.StowPosition;
    RobotStates.intakeState = IntakeState.Stow;
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.AlignCoral;
    return setDone();
  }
}
