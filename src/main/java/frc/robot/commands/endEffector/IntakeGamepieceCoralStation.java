// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.RobotMode;
import frc.robot.RobotStates;
import frc.robot.subsystems.endEffector.EndEffector;
import java.util.function.Supplier;

public class IntakeGamepieceCoralStation extends Command {
  private static final TunableNumberGroup group = new TunableNumberGroup("IntakeGamepiece");
  private static final LoggedTunableNumber intakingVelocity = group.build("intakingVelocity", -500);
  private final EndEffector endEffector;
  private final Supplier<Pose2d> robotPose;
  private final Trigger gamepieceInRobot =
      new Trigger(() -> RobotStates.coralInEndEffectorNonScoringSide || RobotStates.coralInIntake)
          .debounce(0.5);

  /** Creates a new IntakeDefaultIdleRPM. */
  public IntakeGamepieceCoralStation(EndEffector endEffector, Supplier<Pose2d> robotPose) {
    this.endEffector = endEffector;
    this.robotPose = robotPose;
    addRequirements(endEffector);
    setName("IntakeGamepieceCoralStation");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotStates.coralInEndEffectorNonScoringSide || RobotStates.coralInIntake) {
      endEffector.setPercentOut(0);
      endEffector.setGamepieceInRobot(true);
    } else {
      endEffector.setVelocity(intakingVelocity.get());
      endEffector.setGamepieceInRobot(false);

      // Sim put gamepiece in end effector
      // -1.079x - 1.893y + 2.042
      if (RobotMode.isSimBot()) {
        Pose2d blueAllianceReferencePose = AllianceFlipUtil.flipPoseForAlliance(robotPose.get());
        if (Math.min(0, 0) < 1.0) {}
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gamepieceInRobot.getAsBoolean();
  }

  private double distToHumanPlayerStation(Pose2d pose) {
    return distBetweenPointAndLine(pose, 0, 0, 0);
  }

  /**
   * https://study.com/skill/learn/finding-the-distance-between-a-point-line-given-the-point-the-equation-of-the-line-explanation.html#:~:text=Step%201%3A%20Identify%20the%20point,%2C%20%2C%20and%20are%20real%20numbers.
   *
   * @param pose
   * @param a
   * @param b
   * @param c
   * @return distance
   */
  private double distBetweenPointAndLine(Pose2d pose, double a, double b, double c) {
    return 0;
  }
}
