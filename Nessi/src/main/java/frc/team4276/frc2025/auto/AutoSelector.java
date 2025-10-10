package frc.team4276.frc2025.auto;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.Elastic;
import frc.team4276.util.dashboard.Elastic.Notification;
import frc.team4276.util.dashboard.Elastic.Notification.NotificationLevel;
import frc.team4276.util.drivers.VirtualSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoSelector extends VirtualSubsystem {
  public static enum AutoQuestionResponse {
    EMPTY,
    YES,
    NO,
    MIDDLE,
    LEFT,
    RIGHT,
    FAR,
    CLOSE,
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    L1_LEFT,
    L1_RIGHT,
    L2,
    L3,
    PROCESSOR
  }

  private static final int maxQuestions = 10;

  private static final AutoRoutine defaultRoutine =
      new AutoRoutine("Do Nothing", List.of(), () -> Commands.none());

  private final LoggedDashboardChooser<AutoRoutine> routineChooser;
  private final StringPublisher errorPublisher;
  private final List<StringPublisher> questionPublishers;
  private final List<LoggedDashboardChooser<AutoQuestionResponse>> questionChoosers;

  private static boolean autoChanged = true;

  private final Timer errorNotificationTimer = new Timer();
  private final String validAutoText = "We Happy";
  private String prevErrorMsg = validAutoText;

  private final LoggedNetworkNumber coralInput;
  private final LoggedNetworkNumber delayInput;
  private int prevCoralInput = 1;
  private double prevDelayInput = 0.0;

  private AutoRoutine lastRoutine;
  private List<AutoQuestionResponse> lastResponses =
      List.of(
          AutoQuestionResponse.EMPTY,
          AutoQuestionResponse.EMPTY,
          AutoQuestionResponse.EMPTY,
          AutoQuestionResponse.EMPTY,
          AutoQuestionResponse.EMPTY,
          AutoQuestionResponse.EMPTY,
          AutoQuestionResponse.EMPTY,
          AutoQuestionResponse.EMPTY,
          AutoQuestionResponse.EMPTY,
          AutoQuestionResponse.EMPTY);

  public AutoSelector() {
    routineChooser = new LoggedDashboardChooser<>("Comp/Auto/RoutineChooser");
    routineChooser.addDefaultOption(defaultRoutine.name(), defaultRoutine);
    lastRoutine = defaultRoutine;

    errorPublisher =
        NetworkTableInstance.getDefault().getStringTopic("Comp/Auto/ErrorMsg").publish();
    questionPublishers = new ArrayList<>();
    questionChoosers = new ArrayList<>();
    for (int i = 0; i < maxQuestions; i++) {
      var publisher =
          NetworkTableInstance.getDefault()
              .getStringTopic("Comp/Auto/Question #" + Integer.toString(i + 1))
              .publish();
      publisher.set("NA");
      questionPublishers.add(publisher);
      questionChoosers.add(
          new LoggedDashboardChooser<>(
              "Comp/Auto/Question #" + Integer.toString(i + 1) + " Chooser"));
    }

    coralInput = new LoggedNetworkNumber("Comp/Auto/Coral Input", 1);
    delayInput = new LoggedNetworkNumber("Comp/Auto/Delay", 0.0);

    errorNotificationTimer.restart();
  }

  /** Registers a new auto routine that can be selected. */
  public void addRoutine(String name, Supplier<Command> command) {
    addRoutine(name, List.of(), command);
  }

  /** Registers a new auto routine that can be selected. */
  public void addRoutine(String name, List<AutoQuestion> questions, Supplier<Command> command) {
    routineChooser.addOption(name, new AutoRoutine(name, questions, command));
  }

  /** Returns the selected auto command. */
  public Command getCommand() {
    return prevErrorMsg == validAutoText ? lastRoutine.command().get() : Commands.none();
  }

  /** Returns the name of the selected routine. */
  public String getSelectedName() {
    return lastRoutine.name();
  }

  /** Returns the selected question responses. */
  public List<AutoQuestionResponse> getResponses() {
    return lastResponses;
  }

  public int getCoralInput() {
    return (int) coralInput.get();
  }

  public double getDelayInput() {
    return delayInput.get();
  }

  private boolean wasRed = false;

  public void periodic() {
    // Skip updates when actively running in auto
    if (DriverStation.isAutonomousEnabled() && lastRoutine != null && lastResponses != null) {
      return;
    }

    SmartDashboard.putNumber("Comp/Auto/Num Coral Submitted ", getCoralInput());
    SmartDashboard.putNumber("Comp/Auto/Delay Input Submitted ", getDelayInput());

    // Update the list of questions
    var selectedRoutine = routineChooser.get();
    if (selectedRoutine == null) {
      return;
    }

    if (!selectedRoutine.equals(lastRoutine)) {
      autoChanged = true;
      var questions = selectedRoutine.questions();
      questionChoosers.clear();
      for (int i = 0; i < maxQuestions; i++) {
        questionChoosers.add(
            new LoggedDashboardChooser<>(
                "Comp/Auto/Question #" + Integer.toString(i + 1) + " Chooser"));
        if (i < questions.size()) {
          questionPublishers.get(i).set(questions.get(i).question());
          for (int j = 0; j < questions.get(i).responses().size(); j++) {
            var response = questions.get(i).responses().get(j);
            questionChoosers.get(i).addOption(response.toString(), response);
          }
        } else {
          questionPublishers.get(i).set("");
        }
      }
    }

    // Update the routine and responses
    lastRoutine = selectedRoutine;
    var cachedResponses =
        lastResponses.isEmpty() || autoChanged
            ? List.of(
                AutoQuestionResponse.EMPTY,
                AutoQuestionResponse.EMPTY,
                AutoQuestionResponse.EMPTY,
                AutoQuestionResponse.EMPTY,
                AutoQuestionResponse.EMPTY,
                AutoQuestionResponse.EMPTY,
                AutoQuestionResponse.EMPTY,
                AutoQuestionResponse.EMPTY,
                AutoQuestionResponse.EMPTY,
                AutoQuestionResponse.EMPTY)
            : lastResponses;
    lastResponses = new ArrayList<>();
    for (int i = 0; i < lastRoutine.questions().size(); i++) {
      questionChoosers.get(i).periodic();
      var responseString = questionChoosers.get(i).get();
      if (cachedResponses.get(i) != responseString && responseString != null) {
        autoChanged = true;
      }
      lastResponses.add(
          responseString == null
              ? lastRoutine.questions().get(i).responses().get(0)
              : responseString);
    }

    if (getCoralInput() != prevCoralInput || getDelayInput() != prevDelayInput) {
      autoChanged = true;
    }

    prevCoralInput = getCoralInput();
    prevDelayInput = getDelayInput();

    if (AllianceFlipUtil.shouldFlip() != wasRed) {
      autoChanged = true;
    }

    wasRed = AllianceFlipUtil.shouldFlip();

    String errorText = checkLogic();

    // Scream, whine, complain, b*tch, basically be a baby until valid
    if (errorText != validAutoText) {
      if (prevErrorMsg == validAutoText || errorNotificationTimer.advanceIfElapsed(11.0)) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.WARNING, "Invalid Auto", errorText, 10000));
      }

    } else {
      if (autoChanged) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.INFO, "Auto Confirm", "Auto Is Valid", 3000));
      }

      errorNotificationTimer.restart();
    }

    prevErrorMsg = errorText;
    errorPublisher.set(prevErrorMsg);
    SmartDashboard.putBoolean("Comp/Auto/ErrorStatus", prevErrorMsg == validAutoText);
  }

  private String checkLogic() {
    if (getSelectedName() == "Sandy Eggos Auto") {
      if (getResponses().get(5) == AutoQuestionResponse.G
          && getResponses().get(2) == AutoQuestionResponse.YES) {
        return "Using CLOSE Intake with G Start";
      }
    }

    return validAutoText;
  }

  public static boolean hasAutoChanged() {
    if (autoChanged) {
      autoChanged = false;
      return true;
    }

    return false;
  }

  /** A customizable auto routine associated with a single command. */
  private static final record AutoRoutine(
      String name, List<AutoQuestion> questions, Supplier<Command> command) {}

  /** A question to ask for customizing an auto routine. */
  public static record AutoQuestion(String question, List<AutoQuestionResponse> responses) {}
}
