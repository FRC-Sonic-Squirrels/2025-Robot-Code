// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants.RobotMode;
import frc.robot.RobotStates;
import frc.robot.commands.mechanism.MechanismPositions;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;
import java.util.function.Supplier;

public class IntakeGamepieceCoralStation extends Command {
  private static final LoggerGroup logGroup = LoggerGroup.build("IntakeGamepiece");
  private static final LoggerEntry.Integer log_Stage = logGroup.buildInteger("Stage");

  private final EndEffector endEffector;
  private final Trigger gamepieceInRobot =
      new Trigger(() -> RobotStates.coralInEndEffectorScoringSide || RobotStates.coralInIntake)
          .debounce(0.25);
  private final Trigger simConditions;

  /** Creates a new IntakeGamepieceCoralStation. */
  public IntakeGamepieceCoralStation(
      EndEffector endEffector, Elevator elevator, Arm arm, Supplier<Pose2d> robotPose) {
    this.endEffector = endEffector;
    simConditions =
        new Trigger(
                () -> {
                  Pose2d blueAllianceReferencePose =
                      AllianceFlipUtil.flipPoseForAlliance(robotPose.get());
                  MechanismPosition targetPos = MechanismPositions.coralStationPosition();

                  return Math.min(
                              distToHumanPlayerStation(blueAllianceReferencePose.getTranslation()),
                              distToHumanPlayerStation(
                                  GeometryUtil.flipPoseOnAlliance(blueAllianceReferencePose)
                                      .getTranslation()))
                          < 1.0
                      && elevator.isAtTarget(targetPos.elevatorHeight())
                      && arm.isAtTargetAngle(targetPos.armAngle());
                })
            .debounce(0.5);
    addRequirements(endEffector);
    setName("IntakeGamepieceCoralStation");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotStates.changeEndEffectorIfNotAligning(
        RobotStates.EndEffectorDesiredAction.CoralStationIntake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sim put gamepiece in end effector
    if (RobotMode.isSimBot()) {
      if (simConditions.getAsBoolean()) {
        var endEffectorSim = endEffector.getSim();
        if (endEffectorSim != null) {
          endEffectorSim.scoringSideTofDetecting = true;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotStates.changeEndEffectorIfNotAligning(RobotStates.EndEffectorDesiredAction.Idle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotStates.endEffectorDesiredAction
        != RobotStates.EndEffectorDesiredAction.CoralStationIntake;
  }

  private double distToHumanPlayerStation(Translation2d translation) {
    // -1.079x - 1.893y + 2.042
    return distBetweenPointAndLine(translation, -1.079, -1.893, 2.042);
  }

  /**
   * https://study.com/skill/learn/finding-the-distance-between-a-point-line-given-the-point-the-equation-of-the-line-explanation.html#:~:text=Step%201%3A%20Identify%20the%20point,%2C%20%2C%20and%20are%20real%20numbers.
   *
   * @param translation
   * @param a
   * @param b
   * @param c
   * @return distance
   */
  private double distBetweenPointAndLine(Translation2d translation, double a, double b, double c) {
    return Math.abs(a * translation.getX() + b * translation.getY() + c) / Math.hypot(a, b);
  }
}
