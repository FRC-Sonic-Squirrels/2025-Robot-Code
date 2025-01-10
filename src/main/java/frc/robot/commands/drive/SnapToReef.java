// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapToReef extends Command {
  private final Supplier<Translation2d[]> tagOffsets;
  private final DrivetrainWrapper wrapper;

  private final LoggerGroup log_group = LoggerGroup.build("SnapToReef");
  private final LoggerEntry.Decimal log_dist = log_group.buildDecimal("dist");

  private static final TunableNumberGroup group = new TunableNumberGroup("SnapToReef");
  private static final LoggedTunableNumber xkP = group.build("xkP", 0.1);
  private static final LoggedTunableNumber xkI = group.build("xkI", 0);
  private static final LoggedTunableNumber xkD = group.build("xkD", 0.01);

  private static final LoggedTunableNumber ykP = group.build("ykP", 0.01);
  private static final LoggedTunableNumber ykI = group.build("ykI", 0);
  private static final LoggedTunableNumber ykD = group.build("ykD", 0);

  private final PIDController horizPID = new PIDController(xkP.get(), xkD.get(), xkD.get());
  private final PIDController vertPID = new PIDController(ykP.get(), ykI.get(), ykD.get());

  private final Supplier<Distance> distToWall;

  private final int camID;

  /** Creates a new snapToReef. */
  public SnapToReef(
      Supplier<Translation2d[]> tagOffsets,
      DrivetrainWrapper wrapper,
      Supplier<Distance> distToWall,
      int camID) {
    this.tagOffsets = tagOffsets;
    this.wrapper = wrapper;
    this.distToWall = distToWall;
    this.camID = camID;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int hc = hashCode();
    if (xkP.hasChanged(hc) || xkI.hasChanged(hc) || xkD.hasChanged(hc))
      horizPID.setPID(xkP.get(), xkI.get(), xkD.get());
    if (ykP.hasChanged(hc) || ykI.hasChanged(hc) || ykD.hasChanged(hc))
      vertPID.setPID(ykP.get(), ykI.get(), ykD.get());

    Distance dist = distToWall.get();

    log_dist.info(dist.in(Units.Inches));

    wrapper.setVelocityOverride(
        new ChassisSpeeds(
            -vertPID.calculate(dist.in(Units.Inches), 7),
            -horizPID.calculate(tagOffsets.get()[camID].getX(), 0),
            0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrapper.resetVelocityOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
