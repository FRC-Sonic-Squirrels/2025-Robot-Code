// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.endEffector.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCoral extends Command {
  /** Creates a new AlignCoral. */
  private static final LoggerGroup logGroup = LoggerGroup.build("AlignCoral");

  private static final LoggerEntry.Integer log_Stage = logGroup.buildInteger("Stage");

  private static final TunableNumberGroup group = new TunableNumberGroup("AlignCoral");

  private static final LoggedTunableNumber correctionVelocity =
      group.build("correctionVelocity", 400);
  private static final LoggedTunableNumber maxTurns = group.build("maxTurns", 400);

  private EndEffector endEffector;
  private double initialPosition;
  private int stage;
  private boolean shouldEnd = false;

  public AlignCoral(EndEffector endEffector) {
    this.endEffector = endEffector;

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // minDist>aligned>maxDist
    stage = 0;
    shouldEnd = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (stage) {
        // no alignment
      case 0:
        endEffector.setVelocity(correctionVelocity.get());
        if (endEffector.scoringSideTofSeenGamepiece()
            && endEffector.nonScoringSideTOFSeenGamepiece()) {
          stage = 1;
        }
        break;
        // both tof have seen it
      case 1:
        endEffector.setVelocity(-correctionVelocity.get());
        if (!endEffector.nonScoringSideTOFSeenGamepiece()
            && endEffector.scoringSideTofSeenGamepiece()) {
          stage = 2;
        }
        break;
        // moved out of nonScoring tof
      case 2:
        endEffector.setVelocity(correctionVelocity.get());
        if (endEffector.nonScoringSideTOFSeenGamepiece()) {
          stage = 3;
          initialPosition = endEffector.getMotorPosition();
        }
        break;
        // moved back in
      case 3:
        endEffector.setVelocity(-correctionVelocity.get());
        if (Math.abs(endEffector.getMotorPosition() - initialPosition) >= maxTurns.get()) {
          shouldEnd = true;
        }
        break;
      default:
        shouldEnd = true;
        break;
    }
    log_Stage.info(stage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd
        || !endEffector.nonScoringSideTOFSeenGamepiece()
            && !endEffector.scoringSideTofSeenGamepiece();
  }
}
