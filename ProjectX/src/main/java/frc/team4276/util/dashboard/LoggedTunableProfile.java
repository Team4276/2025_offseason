package frc.team4276.util.dashboard;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.team4276.frc2025.Constants;

public class LoggedTunableProfile {
  private TrapezoidProfile profile;
  public final LoggedTunableNumber maxVel;
  public final LoggedTunableNumber maxAccel;

  private final String key;

  public LoggedTunableProfile(String key, double maxVel, double maxAccel) {
    profile = new TrapezoidProfile(new Constraints(maxVel, maxAccel));
    this.key = key;
    this.maxVel = new LoggedTunableNumber(this.key + "/MaxVel", maxVel);
    this.maxAccel = new LoggedTunableNumber(this.key + "/MaxAccel", maxAccel);
  }

  public State calculate(double t, State current, State goal) {
    if (Constants.isTuning) {
      profile = new TrapezoidProfile(new Constraints(maxVel.getAsDouble(), maxAccel.getAsDouble()));
    }

    return profile.calculate(t, current, goal);
  }
}
