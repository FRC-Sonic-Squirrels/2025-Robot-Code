// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.GeometryUtil;
import frc.robot.RobotStates;
import frc.robot.RobotStates.MechState;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetMechToIntakePosition extends Command {

  private final DrivetrainWrapper wrapper;
  private final Elevator elevator;
  private final Arm arm;

  private Pose2d initPose;

  /** Creates a new ResetMechToScorePrepPosition. */
  public ResetMechToIntakePosition(DrivetrainWrapper wrapper, Elevator elevator, Arm arm) {
    this.wrapper = wrapper;
    this.elevator = elevator;
    this.arm = arm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPose = wrapper.getRawOdometryPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotStates.mechState = MechState.CoralStationPosition;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return GeometryUtil.getDist(initPose, wrapper.getRawOdometryPose()) > 0.3;
  }
}
