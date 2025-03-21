package frc.lib.team2930.lib.controller_rumble;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Consumer;

public class ControllerRumbleForTime extends Command {
  private Consumer<Double> rumbleConsumer;
  private double rumbleStrength;
  private double timeSeconds;
  private Timer timer = new Timer();

  /** Rumble the controller until the specified button is pressed */
  public ControllerRumbleForTime(
      Consumer<Double> rumbleConsumer, double timeSeconds, double rumbleStrength) {
    this.rumbleConsumer = rumbleConsumer;
    this.rumbleStrength = rumbleStrength;
    this.timeSeconds = timeSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rumbleConsumer.accept(rumbleStrength);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rumbleConsumer.accept(0.0);
    timer.reset();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > timeSeconds;
  }
}
