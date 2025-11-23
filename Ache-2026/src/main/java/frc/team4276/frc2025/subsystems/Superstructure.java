package frc.team4276.frc2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.elevator.Elevator;
import frc.team4276.frc2025.subsystems.intake.Intake;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.lib.hid.ViXController;

public class Superstructure extends SubsystemBase {
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Elevator elevator;
  private final Intake intake;
  private final ViXController controller;

  public enum WantedSuperState {

  }

  public enum CurrentSuperState {

  }

  public Superstructure(Drive drive, Vision vision, Elevator elevator, Intake intake, ViXController controller){
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.intake = intake;
    this.controller = controller;
  }

  @Override
  public void periodic() {
      
  }
}
