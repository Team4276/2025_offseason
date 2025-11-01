package frc.team4276.frc2025.subsystems.endeffector;

import frc.team4276.util.dashboard.LoggedTunableNumber;

public class EndEffectorConstants {
  public static final LoggedTunableNumber favorVolts =
      new LoggedTunableNumber("EndEffector/FavorVolts", 4.5);
  public static final LoggedTunableNumber lagVolts =
      new LoggedTunableNumber("EndEffector/LagVolts", 2.0);
  public static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("EndEffector/IntakeVolts", 5.0);
  public static final LoggedTunableNumber intakeSlowVolts =
      new LoggedTunableNumber("EndEffector/SlowIntakeVolts", 3.0);
  public static final LoggedTunableNumber scoreVolts =
      new LoggedTunableNumber("EndEffector/ScoreVolts", 6.0);
  public static final LoggedTunableNumber reverseVolts =
      new LoggedTunableNumber("EndEffector/ReverseVolts", -1.0);
  public static final LoggedTunableNumber purgeVolts =
      new LoggedTunableNumber("EndEffector/PurgeVolts", 7.0);
}
