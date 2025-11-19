package frc.team4276.frc2025.subsystems;

import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.elevator.Elevator;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.lib.hid.ViXController;

public class Superstructure {
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Elevator elevator;
  private final ViXController controller = new ViXController(0);

  public Superstructure(Drive drive, Vision vision, Elevator elevator, ViXController controller){
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
  }
}
