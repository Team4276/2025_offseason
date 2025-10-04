package frc.team4276.util.dashboard;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class LoggedTunableFF {
  public enum FeedforwardType {
    SIMPLE,
    LINEAR,
    ARM
  }

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0, 0);

  private final String key;
  private final FeedforwardType type;

  private LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.20);
  private LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 7.8);
  private LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.15);
  private LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.01);

  public LoggedTunableFF(String key, FeedforwardType type) {
    this.key = key;
    this.type = type;
  }
}
