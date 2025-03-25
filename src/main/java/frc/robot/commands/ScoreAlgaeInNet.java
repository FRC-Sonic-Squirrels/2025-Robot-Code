package frc.robot.commands;

import frc.robot.commands.ScoreCoral.ScoringSide;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.Optional;
import java.util.function.Consumer;

public class ScoreAlgaeInNet {

  private final DrivetrainWrapper wrapper;
  private final Mechanism mech;
  private final Elevator elevator;
  private final Arm arm;
  private final LED led;
  private final RobotConfig config;
  private final Consumer<Double> rumble;
  private final boolean confirmation;

  public ScoreAlgaeInNet(
      DrivetrainWrapper wrapper,
      Mechanism mech,
      Elevator elevator,
      Arm arm,
      LED led,
      Consumer<Double> rumble,
      RobotConfig config) {
    this(wrapper, mech, elevator, arm, led, rumble, config);
  }

  private boolean preloadCode;

  public ScoreAlgaeInNet(
      DrivetrainWrapper wrapper,
      Mechanism mech,
      Elevator elevator,
      Arm arm,
      LED led,
      Consumer<Double> rumble,
      RobotConfig config,
      boolean preloadCode) {
    this(wrapper, mech, elevator, arm, led, rumble, config);
    this.preloadCode = preloadCode;
  }

  public ScoreAlgaeInNet(
      DrivetrainWrapper wrapper,
      Mechanism mech,
      Elevator elevator,
      Arm arm,
      LED led,
      Optional<ScoringSide> side,
      Consumer<Double> rumble,
      RobotConfig config,
      boolean driverConfirmation) {

    this.wrapper = wrapper;
    this.mech = mech;
    this.elevator = elevator;
    this.arm = arm;
    this.led = led;
    this.config = config;

    this.rumble = rumble;
    confirmation = driverConfirmation;
  }
}
