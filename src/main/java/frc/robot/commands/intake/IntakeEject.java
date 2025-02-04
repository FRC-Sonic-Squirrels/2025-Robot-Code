// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeEject extends Command {
  private Intake intake;

  private static final TunableNumberGroup group = new TunableNumberGroup("EjectGamepiece");
  private static final LoggedTunableNumber ejectingVelocity =
      group.build("ejectingVelocity", 2500.0);

  /** Creates a new IntakeEject. */
  public IntakeEject(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // TODO: test on 2025 bot
  @Override
  public void execute() {
    intake.setPivotAngle(PivotConstants.MAX_PIVOT_ANGLE);
    if (intake.isPivotAtTargetAngle()) {
      intake.setRollerVelocity(ejectingVelocity.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollerPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
