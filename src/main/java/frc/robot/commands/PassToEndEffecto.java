package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.StateMachine;
import frc.robot.RobotStates;
import frc.robot.RobotStates.EndEffectorDesiredAction;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.MechState;

public class PassToEndEffecto extends StateMachine {

  private final Trigger endTrigger =
      new Trigger(() -> RobotStates.coralInEndEffectorScoringSide).debounce(0.3);

  public PassToEndEffecto() {
    super("PassToEndEffector");

    setInitialState(stateWithName("PrepToPassOff", () -> prepToPassOff()));
    setInterruptedState(stateWithName("End", () -> end()));
  }

  private StateHandler prepToPassOff() {
    RobotStates.mechanismInUse = true;
    RobotStates.intakeInUse = true;
    RobotStates.endEffectorInUse = true;
    RobotStates.mechState = MechState.PrepPassoffPosition;
    RobotStates.intakeState = IntakeState.PrepPassoff;
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.PassToEndEffector;
    return RobotStates.intakeInTargetState && RobotStates.mechInTargetState
        ? stateWithName("PassOff", () -> passOff())
        : null;
  }

  private StateHandler passOff() {
    RobotStates.mechanismInUse = true;
    RobotStates.intakeInUse = true;
    RobotStates.endEffectorInUse = true;
    RobotStates.mechState = MechState.PassoffPosition;
    RobotStates.intakeState = IntakeState.PassoffEndEffector;
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.PassToEndEffector;
    return endTrigger.getAsBoolean() ? () -> end() : null;
  }

  private StateHandler end() {
    RobotStates.mechState = MechState.StowPosition;
    RobotStates.intakeState = IntakeState.Stow;
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.AlignCoral;
    RobotStates.mechanismInUse = false;
    RobotStates.intakeInUse = false;
    RobotStates.endEffectorInUse = false;
    return setDone();
  }
}
