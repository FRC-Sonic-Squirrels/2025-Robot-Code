// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.helpers.ChoreoHelper.ChassisSpeedsWithPathEnd;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowPath extends Command {

  private final DrivetrainWrapper wrapper;
  private final Supplier<Pose2d> currentPose;
  private final RobotConfig config;
  private final Trajectory<SwerveSample> traj;

  private static LoggerGroup logGroup = LoggerGroup.build("FollowPath");
  private static LoggerEntry.Struct<Pose2d> log_targetChassisSpeeds =
      logGroup.buildStruct(Pose2d.class, "ClearAlgae");

  private ChoreoHelper helper;
  private boolean pathFinished = false;

  private double finalOffsetErrorLimit = Double.NaN;
  private double finalTargetErrorLimit = Double.NaN;
  private double finalHeadingErrorLimit = Double.NaN;
  private double finalErrorMaxWait = 2;

  /** Creates a new DriveToPosePathing. */
  public FollowPath(
      DrivetrainWrapper wrapper,
      Supplier<Pose2d> currentPose,
      RobotConfig config,
      Trajectory<SwerveSample> traj) {
    this.wrapper = wrapper;
    this.currentPose = currentPose;
    this.config = config;
    this.traj = traj;
  }

  public FollowPath setFinalOffsetError(double maxError) {
    this.finalOffsetErrorLimit = maxError;
    return this;
  }

  public FollowPath setFinalTargetError(double maxError) {
    this.finalTargetErrorLimit = maxError;
    return this;
  }

  public FollowPath setFinalHeadingError(double maxError) {
    this.finalHeadingErrorLimit = maxError;
    return this;
  }

  public FollowPath setFinalErrorMaxWait(double maxWait) {
    this.finalErrorMaxWait = maxWait;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initializeNoCheck();
  }

  public void initializeNoCheck() {
    if (traj == null) {
      this.cancel();
      return;
    }

    helper =
        new ChoreoHelper(
            Timer.getFPGATimestamp(),
            traj.getInitialPose(false).orElseThrow(),
            new ChoreoTrajectoryWithName("DriveToPose", traj),
            config.getDriveBaseRadius() / 2,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());

    helper.setFinalErrorMaxWait(finalErrorMaxWait);
    helper.setFinalOffsetError(finalOffsetErrorLimit);
    helper.setFinalTargetError(finalTargetErrorLimit);
    helper.setFinalHeadingError(finalHeadingErrorLimit);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (helper == null) {
      pathFinished = true;
      return;
    }

    ChassisSpeedsWithPathEnd result =
        helper.calculateChassisSpeeds(currentPose.get(), Timer.getFPGATimestamp());
    log_targetChassisSpeeds.info(
        new Pose2d(
            result.chassisSpeeds().vxMetersPerSecond,
            result.chassisSpeeds().vyMetersPerSecond,
            Rotation2d.fromRadians(result.chassisSpeeds().omegaRadiansPerSecond)));
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
