// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.RobotStates.MechState;
import frc.robot.subsystems.mechanism.Mechanism;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MechToPosition extends Command {

  private final MechState state;
  private final Mechanism mech;
  private final RobotStates states;

  /** Creates a new MechToPosition. */
  public MechToPosition(Mechanism mech, MechState state, RobotStates states) {
    this.mech = mech;
    this.state = state;
    this.states = states;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    states.mechanismInUse = true;
    states.mechState = state;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    states.mechanismInUse = false;
    return mech.mechInPosition();
  }
}
