package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotStates.MechState;
import frc.robot.commands.endEffector.IntakeGamepieceCoralStation;
import frc.robot.commands.mechanism.MechToPosition;
import frc.robot.commands.mechanism.WaitUntilMovedDist;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class CommandComposer {
  public static Command intakeCoralFromStation(
      DrivetrainWrapper wrapper,
      EndEffector endEffector,
      Mechanism mech,
      LED led,
      XboxControllerWrapper driverController,
      boolean moveMech) {

    return new IntakeGamepieceCoralStation(
            endEffector, mech, () -> wrapper.getCoralStationPoseEstimatorPose(true))
        .alongWith(
            Commands.run(
                    () -> {
                      if (RobotStates.coralInEndEffector) {
                        if (driverController != null)
                          driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
                        // led.setBaseRobotState(BaseRobotState.INTAKE_SUCCESS);
                      }
                    })
                .finallyDo(
                    () -> {
                      if (driverController != null)
                        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                      led.setBaseRobotState(BaseRobotState.LEVEL_MODE);
                    }))
        .andThen(
            new WaitUntilMovedDist(wrapper, Units.Meters.of(0.3))
                .andThen(
                    new MechToPosition(mech, MechState.StowPosition)
                        .finallyDo(
                            () -> {
                              RobotStates.changeEndEffectorIfNotAligning(
                                  RobotStates.EndEffectorDesiredAction.AlignCoral);
                            })))
        .withName("CoralStationIntake");
  }
}
