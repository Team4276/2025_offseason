package frc.team4276.frc2025.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.drivers.VirtualSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoSelector extends VirtualSubsystem {
  private final AutoFactory autoFactory;

  private final LoggedDashboardChooser<Supplier<Command>> routineChooser =
      new LoggedDashboardChooser<>("Comp/Auto/RoutineChooser");
  private Supplier<Command> lastRoutine = () -> Commands.none();

  private static boolean autoChanged = true;

  private final LoggedNetworkNumber delayInput = new LoggedNetworkNumber("Comp/Auto/Delay", 0.0);

  public AutoSelector(AutoFactory autoFactory) {
    this.autoFactory = autoFactory;

    routineChooser.addDefaultOption("Do Nothing", () -> this.autoFactory.idle());
    routineChooser.addOption("Taxi Right", () -> this.autoFactory.taxiCommand(false));
    routineChooser.addOption("Taxi Left", () -> this.autoFactory.taxiCommand(true));
    routineChooser.addOption("Taxi Mid Right", () -> this.autoFactory.taxiMidCommand(false));
    routineChooser.addOption("Taxi Mid Left", () -> this.autoFactory.taxiMidCommand(false));
    routineChooser.addOption("3x Right: EBA", () -> this.autoFactory.EBA());
    routineChooser.addOption("3x Left: JAB", () -> this.autoFactory.JAB());
    routineChooser.addOption("4x Right: FEDC", () -> this.autoFactory.FEDC());
    routineChooser.addOption("4x Left: IJKL", () -> this.autoFactory.IJKL());
    routineChooser.addOption("Jitb 4x Right", () -> this.autoFactory.jitbProcessorSide());
    routineChooser.addOption("Jitb 4x Left", () -> this.autoFactory.jitbBargeSide());
    routineChooser.addOption("5x Right: JAB", () -> this.autoFactory.poofsProcessorSide());
    routineChooser.addOption("5x Left: ECDCD", () -> this.autoFactory.poofsBargeSide());
  }

  /** Returns the selected auto command with the inputted delay. */
  public Command getCommand() {
    return lastRoutine
        .get()
        .beforeStarting(Commands.waitSeconds(getDelayInput()))
        .finallyDo(() -> this.autoFactory.autoEnd());
  }

  public double getDelayInput() {
    return delayInput.get();
  }

  private boolean wasRed = false;

  public void periodic() {
    // Skip updates when actively running in auto
    if (DriverStation.isAutonomousEnabled() && lastRoutine != null) {
      return;
    }

    SmartDashboard.putNumber("Comp/Auto/Delay Input Submitted ", getDelayInput());

    // Update the list of questions
    var selectedRoutine = routineChooser.get();
    if (selectedRoutine == null) {
      return;
    }

    // Update the routine and responses
    if (lastRoutine != selectedRoutine.get()) {
      lastRoutine = selectedRoutine;
      autoChanged = true;
    }

    if (AllianceFlipUtil.shouldFlip() != wasRed) {
      autoChanged = true;
    }

    wasRed = AllianceFlipUtil.shouldFlip();
  }

  public static boolean hasAutoChanged() {
    if (autoChanged) {
      autoChanged = false;
      return true;
    }

    return false;
  }
}
