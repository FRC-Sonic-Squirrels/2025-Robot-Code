// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.endEffector.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCoral extends Command {
  /** Creates a new AlignCoral. */
  private static final TunableNumberGroup group = new TunableNumberGroup("AlignCoral");

  private static final LoggedTunableNumber correctionVelocity =
      group.build("correctionVelocity", 800);
  private static final LoggedTunableNumber minTOFDistanceInches =
      group.build("minTOFDistanceInches", 3);
  private static final LoggedTunableNumber maxTOFDistanceInches =
      group.build("maxTOFDistanceInches", 5);

  EndEffector endEffector;
  boolean aligned;
  boolean intitalMovementDone = false;
  boolean shouldEnd = false;

  public AlignCoral(EndEffector endEffector) {
    this.endEffector = endEffector;

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // minDist>aligned>maxDist
    aligned =
        endEffector.scoringSideTofDistance().lt(Units.Inches.of(maxTOFDistanceInches.get()))
            && endEffector.scoringSideTofDistance().gt(Units.Inches.of(minTOFDistanceInches.get()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (aligned) {
      if (intitalMovementDone) {
        endEffector.setVelocity(-correctionVelocity.get());
      } else {
        endEffector.setVelocity(correctionVelocity.get());
        intitalMovementDone = !endEffector.scoringSideTofSeenGamepiece();
      }
      shouldEnd = intitalMovementDone && endEffector.scoringSideTofSeenGamepiece();
    } else {
      endEffector.setVelocity(correctionVelocity.get());
      shouldEnd =
          !endEffector.scoringSideTofSeenGamepiece()
              && !endEffector.nonScoringSideTOFSeenGamepiece();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd;
  }
}
