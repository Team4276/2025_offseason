package frc.team4276.frc2025;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.team4276.frc2025.field.FieldConstants.Reef;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure.Goal;
import frc.team4276.util.ViXController;
import frc.team4276.util.drivers.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class ScoringHelper extends VirtualSubsystem {
  private final CommandGenericHID buttonBoard;
  private final ViXController xbox;

  private Reef reef = Reef.A;
  private Goal goal = Goal.L1;

  public ScoringHelper(CommandGenericHID buttonBoard, ViXController xbox) {
    this.buttonBoard = buttonBoard;
    this.xbox = xbox;
  }

  @Override
  public void periodic() {
    updateXbox(); // redundancy
    updateButtonBoard();

    SmartDashboard.putBoolean("Comp/ScoringHelper/L3", goal == Goal.L3);
    SmartDashboard.putBoolean("Comp/ScoringHelper/L2", goal == Goal.L2);
    SmartDashboard.putBoolean("Comp/ScoringHelper/L1", goal == Goal.L1);

    Logger.recordOutput("ScoringHelper/SelectedAlignPose", getSelectedReef().getAlign());
    Logger.recordOutput("ScoringHelper/SelectedScorePose", getSelectedReef().getScore());
    Logger.recordOutput("ScoringHelper/Goal", goal);
    Logger.recordOutput("ScoringHelper/SelectedReef", getSelectedReef());
  }

  private void updateButtonBoard() {
    // Update Positions
    if (buttonBoard.getHID().getRawButtonPressed(6)) {
      reef = Reef.A;
    } else if (buttonBoard.getHID().getRawButtonPressed(5)) {
      reef = Reef.B;
    } else if (buttonBoard.getHID().getRawButtonPressed(4)) {
      reef = Reef.C;
    } else if (buttonBoard.getHID().getRawButtonPressed(3)) {
      reef = Reef.D;
    } else if (buttonBoard.getHID().getRawButtonPressed(2)) {
      reef = Reef.E;
    } else if (buttonBoard.getHID().getRawButtonPressed(1)) {
      reef = Reef.F;
    } else if (buttonBoard.getHID().getPOV() == 90) {
      reef = Reef.G;
    } else if (buttonBoard.getHID().getPOV() == 270) {
      reef = Reef.H;
    } else if (buttonBoard.getHID().getPOV() == 180) {
      reef = Reef.I;
    } else if (buttonBoard.getHID().getPOV() == 0) {
      reef = Reef.J;
    } else if (buttonBoard.getHID().getRawButtonPressed(8)) {
      reef = Reef.K;
    } else if (buttonBoard.getHID().getRawButtonPressed(7)) {
      reef = Reef.L;
    }

    // Update Level
    if (buttonBoard.getRawAxis(2) == 1) {
      goal = Goal.L1;
    } else if (buttonBoard.getRawAxis(3) == 1) {
      goal = Goal.L2;
    } else if (buttonBoard.getHID().getRawButtonPressed(9)) {
      goal = Goal.L3;
    }
  }

  private boolean isRight = false;
  private int side = 0;

  private void updateXbox() {
    boolean prevRight = isRight;
    int prevSide = side;

    // Update Positions
    if (xbox.getHID().getLeftBumperButton()) {
      isRight = false;
    } else if (xbox.getHID().getRightBumperButton()) {
      isRight = true;
    }

    if (xbox.getHID().getPOV() == 180) {
      side = 0;
    } else if (xbox.getHID().getPOV() == 135) {
      side = 1;
    } else if (xbox.getHID().getPOV() == 45) {
      side = 2;
    } else if (xbox.getHID().getPOV() == 0) {
      side = 3;
    } else if (xbox.getHID().getPOV() == 315) {
      side = 4;
    } else if (xbox.getHID().getPOV() == 225) {
      side = 5;
    }

    // Update Level
    if (xbox.getHID().getAButton()) {
      goal = Goal.L1;
    } else if (xbox.getHID().getBButton()) {
      goal = Goal.L2;
    } else if (xbox.getHID().getYButton()) {
      goal = Goal.L3;
    }

    if (prevSide != side || prevRight != isRight) {
      if (side > 1 && side < 5) {
        reef = Reef.values()[(side * 2) + (isRight ? 0 : 1)];

      } else {
        reef = Reef.values()[(side * 2) + (isRight ? 1 : 0)];
      }
    }
  }

  public Goal getSuperstructureGoal() {
    return goal;
  }

  public Reef getSelectedReef() {
    return reef;
  }
}
