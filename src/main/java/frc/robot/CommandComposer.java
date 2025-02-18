package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.endEffector.IntakeGamepieceCoralStation;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class CommandComposer {
  private static Pose2d initPose;

  public static Command intakeCoralFromStation(
      DrivetrainWrapper wrapper,
      EndEffector endEffector,
      Elevator elevator,
      Arm arm,
      LED led,
      XboxControllerWrapper driverController) {

    return Commands.runOnce(
            () -> {
              initPose = wrapper.getRawOdometryPose();
            })
        .andThen(
            MechanismActions.coralStationPosition(elevator, arm)
                .alongWith(
                    new IntakeGamepieceCoralStation(
                        endEffector,
                        elevator,
                        arm,
                        () -> wrapper.getCoralStationPoseEstimatorPose(true)))
                // .andThen(
                //     Commands.waitUntil(
                //         () -> GeometryUtil.getDist(initPose, wrapper.getRawOdometryPose()) >
                // 0.3))
                // .andThen(MechanismActions.scorePrepPosition(elevator, arm))
                .alongWith(
                    Commands.run(
                            () -> {
                              if (endEffector
                                  .isGamepieceFullyInEndEffector()) { // If the gamepiece is in
                                // robot, set
                                // rumble
                                if (driverController != null)
                                  driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
                                led.setBaseRobotState(BaseRobotState.INTAKE_SUCCESS);
                              }
                            })
                        .finallyDo(
                            () -> {
                              if (driverController != null)
                                driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                              led.setBaseRobotState(BaseRobotState.LEVEL_MODE);
                            })));
  }
}
