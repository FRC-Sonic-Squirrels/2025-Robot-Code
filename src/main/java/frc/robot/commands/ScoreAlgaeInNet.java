package frc.robot.commands;

import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Consumer;

public class ScoreAlgaeInNet {
  public ScoreAlgaeInNet(
      DrivetrainWrapper wrapper,
      Mechanism mech,
      Elevator elevator,
      Arm arm,
      LED led,
      Consumer<Double> rumble,
      RobotConfig config) {}
}
