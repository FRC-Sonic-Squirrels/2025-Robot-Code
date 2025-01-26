// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.records.AutoDescriptor;

public class AutoStateMachine extends StateMachine {

  /** Creates a new AutoSubstateMachine. */
  public AutoStateMachine(AutosSubsystems subsystems, AutoDescriptor descriptors) {
    super("Auto");
  }
}
