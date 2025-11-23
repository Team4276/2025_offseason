package frc.team4276.frc2025.subsystems;

import org.littletonrobotics.junction.Logger;

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
    STOW,
    REEF_TEST,
    INTAKE_CORAL,
    SCORE_L1_INTAKE,
    PURGE
  }

  public enum CurrentSuperState {
    STOWED,
    REEF_TESTING,
    INTAKING_CORAL,
    SCORING_L1_INTAKE,
    PURGING
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOW;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOWED;

  public enum GamePieceState {
    NO_BANANA,
    CORAL,
    ALGAE
  }

  private GamePieceState gamePieceState = GamePieceState.NO_BANANA;

  private boolean isL1Mode = true;
  
  public Superstructure(Drive drive, Vision vision, Elevator elevator, Intake intake, ViXController controller){
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.intake = intake;
    this.controller = controller;
  }

  @Override
  public void periodic() {
    
      if(intake.hasCoral()){

      } else {
        gamePieceState = GamePieceState.NO_BANANA;

      }

    currentSuperState = handleStateTransition();
    applyState();

    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
  }

  private CurrentSuperState handleStateTransition(){
    return switch (wantedSuperState) {
      case STOW: 
        yield CurrentSuperState.STOWED;
      case REEF_TEST: 
        yield CurrentSuperState.REEF_TESTING;
      case INTAKE_CORAL: 
        yield CurrentSuperState.INTAKING_CORAL;
      case SCORE_L1_INTAKE: 
        yield CurrentSuperState.SCORING_L1_INTAKE;
      case PURGE: 
        yield CurrentSuperState.PURGING;
    };
  }

  private void applyState(){

  }
}
