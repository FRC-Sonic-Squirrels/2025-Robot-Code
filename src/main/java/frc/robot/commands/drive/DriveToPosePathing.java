// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TrajectoryUtil;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.helpers.ChoreoHelper.ChassisSpeedsWithPathEnd;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.List;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPosePathing extends Command {

  private final DrivetrainWrapper wrapper;
  private final RobotConfig config;
  private final Supplier<Pose2d> currentPose;
  private final Supplier<Pose2d> targetPose;
  private final Rotation2d endDirectionOfTravel;

  private ChoreoHelper helper;
  private boolean pathFinished = false;

  /** Creates a new DriveToPosePathing. */
  public DriveToPosePathing(
      DrivetrainWrapper wrapper,
      RobotConfig config,
      Supplier<Pose2d> currentPose,
      Supplier<Pose2d> targetPose,
      Rotation2d endDirectionOfTravel) {
    this.wrapper = wrapper;
    this.config = config;
    this.currentPose = currentPose;
    this.targetPose = targetPose;
    this.endDirectionOfTravel = endDirectionOfTravel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d initPose = currentPose.get();

    Translation2d vel = wrapper.getFieldRelativeVelocities().getTranslation();

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(initPose.getTranslation(), new Rotation2d(vel.getX(), vel.getY())),
            new Pose2d(targetPose.get().getTranslation(), endDirectionOfTravel));

    PathConstraints constraints =
        new PathConstraints(
            config.getPathingMaxSpeedMPS().get(),
            config.getPathingMaxAccelerationMPSPS().get(),
            config.getPathingMaxAngularVelocityRadPerSecond().get(),
            config.getPathingMaxAngularAccelerationRadPerSecondSquared().get());

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                targetPose
                    .get()
                    .getRotation()) // Goal end state. You can set a holonomic rotation here. If
            // using a
            // differential drivetrain, the rotation will have no effect.
            );

    PathPlannerTrajectory traj =
        path.generateTrajectory(
            wrapper.getCurrentRobotRelativeChassisSpeeds(),
            wrapper.getRotationGyroOnly(),
            config.pathPlannerConfig());

    Trajectory<SwerveSample> choreoTraj = TrajectoryUtil.pathPlannerToChoreo(traj);

    helper =
        new ChoreoHelper(
            Timer.getFPGATimestamp(),
            initPose,
            new ChoreoTrajectoryWithName("DriveToPose", choreoTraj),
            config.getDriveBaseRadius() / 2,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeedsWithPathEnd result =
        helper.calculateChassisSpeeds(currentPose.get(), Timer.getFPGATimestamp());
    wrapper.setVelocityOverride(result.chassisSpeeds());
    pathFinished = result.atEndOfPath();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrapper.resetVelocityOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathFinished;
  }
}
