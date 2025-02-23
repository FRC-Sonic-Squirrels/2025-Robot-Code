// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  /** Creates a new Climb. */
  private static final LoggerGroup logGroup = LoggerGroup.build("Climb");

  private static final TunableNumberGroup group = new TunableNumberGroup("Climb");

  private static final LoggedTunableNumber winchSpeed = group.build("winchSpeed", 1.0);

  private Climber climber;

  private boolean shouldEnd;

  public Climb(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shouldEnd = false;
    // servo cannot sense its angle so just assume its there
    climber.setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setWinchVoltage(-winchSpeed.get());
    shouldEnd =
        Math.abs(
                climber.getWinchAngle().getRotations()
                    - ClimberConstants.CLIMB_WINCH_ROTATIONS.getRotations())
            <= 0.1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd;
  }
}
