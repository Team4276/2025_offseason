// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2025;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.lib.hid.ViXController;

public class RobotContainer {
  private final ViXController mDriver = new ViXController(Ports.DRIVER_CONTROLLER);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
