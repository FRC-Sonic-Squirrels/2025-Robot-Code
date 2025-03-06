package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class XboxControllerWrapper {
  public enum Button {
    a(CommandXboxController::a),
    b(CommandXboxController::b),
    x(CommandXboxController::x),
    y(CommandXboxController::y),
    back(CommandXboxController::back),
    start(CommandXboxController::start),
    leftStick(CommandXboxController::leftStick),
    rightStick(CommandXboxController::rightStick),
    leftBumper(CommandXboxController::leftBumper),
    rightBumper(CommandXboxController::rightBumper),
    leftTrigger(CommandXboxController::leftTrigger),
    rightTrigger(CommandXboxController::rightTrigger),
    povUp(CommandXboxController::povUp),
    povDown(CommandXboxController::povDown),
    povLeft(CommandXboxController::povLeft),
    povRight(CommandXboxController::povRight);

    final Function<CommandXboxController, Trigger> binder;

    Button(Function<CommandXboxController, Trigger> binder) {
      this.binder = binder;
    }

    Trigger bind(CommandXboxController controller) {
      return binder.apply(controller);
    }
  }

  private final CommandXboxController controller;
  private final Map<Button, LoggerEntry.Text> buttons = new HashMap<>();

  public XboxControllerWrapper(int port) {
    LoggerGroup logGroup =
        switch (port) {
          case 0 -> {
            controller = new CommandXboxController(0);
            yield LoggerGroup.build("XboxDriver");
          }
          case 1 -> {
            controller = new CommandXboxController(1);
            yield LoggerGroup.build("XboxOperator");
          }
          default -> throw new RuntimeException("Invalid controller port: " + port);
        };

    for (Button button : Button.values()) {
      var entry = logGroup.buildString(button.name());
      entry.info("<unassigned>");

      buttons.put(button, entry);
    }
  }

  public Trigger registerTrigger(Button button, String role) {
    buttons.get(button).info(role);

    return button.bind(controller);
  }

  public GenericHID getHID() {
    return controller.getHID();
  }

  public double getLeftX() {
    return controller.getLeftX();
  }

  public double getLeftY() {
    return controller.getLeftY();
  }

  public double getRightX() {
    return controller.getRightX();
  }

  public double getRightY() {
    return controller.getRightY();
  }

  public Trigger getPovUp() {
    return controller.povUp();
  }

  public Trigger getPovDown() {
    return controller.povDown();
  }

  public Trigger getPovLeft() {
    return controller.povLeft();
  }

  public Trigger getPovRight() {
    return controller.povRight();
  }
}
