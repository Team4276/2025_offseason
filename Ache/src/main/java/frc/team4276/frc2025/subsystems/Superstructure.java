package frc.team4276.frc2025.subsystems;

import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.field.FieldConstants.ScoringSide;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.drive.Drive.WantedState;
import frc.team4276.frc2025.subsystems.elevator.Elevator;
import frc.team4276.frc2025.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.lib.hid.ViXController;

public class Superstructure {
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Elevator elevator;

  private final ViXController controller;

  public Superstructure(Drive drive, Vision vision, Elevator elevator, ViXController controller) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.controller = controller;
  }

  public void testLeft() {
    FieldConstants.getCoralScorePose(
        RobotState.getInstance().getTagIdFromClosestPoseSide(), ScoringSide.LEFT)
        .ifPresent((pose) -> drive.setAutoAlignPose(pose));
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L2);

  }

  public void testRight() {
    FieldConstants.getCoralScorePose(
        RobotState.getInstance().getTagIdFromClosestPoseSide(), ScoringSide.RIGHT)
        .ifPresent((pose) -> drive.setAutoAlignPose(pose));
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L2);

  }

  public void stow() {
    drive.setWantedState(WantedState.TELEOP);
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.STOW);

  }

}
