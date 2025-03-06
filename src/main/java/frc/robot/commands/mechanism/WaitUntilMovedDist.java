// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.GeometryUtil;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitUntilMovedDist extends Command {

  private final DrivetrainWrapper wrapper;
  private final Distance dist;

  private Pose2d initPose;

  /** Creates a new ResetMechToScorePrepPosition. */
  public WaitUntilMovedDist(DrivetrainWrapper wrapper, Distance dist) {
    this.wrapper = wrapper;
    this.dist = dist;

    // Use addRequirements() here to declare subsystem dependencies.
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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return GeometryUtil.getDist(initPose, wrapper.getRawOdometryPose()) > dist.in(Units.Meters);
  }
}
