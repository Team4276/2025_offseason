package frc.team4276.util.drivers;

import edu.wpi.first.wpilibj.Timer;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.stat.StatUtils;
import org.littletonrobotics.junction.Logger;

public class VelocitySensor { // add a debounce?
  private final String key;

  private final LoggedTunableNumber velDipThreshold;
  private final LoggedTunableNumber deccelThreshold;
  private final LoggedTunableNumber upperBoundAccel;
  private final LoggedTunableNumber lowerBoundAccel;

  private final List<Double> peakVelocitySamples = new ArrayList<>();

  private double averageVelocity;
  private double lastVelocity;
  private boolean dipDetected;
  private double timeOld;

  public VelocitySensor(String key) {
    this.key = key;
    timeOld = Timer.getTimestamp();

    velDipThreshold = new LoggedTunableNumber(key + "/VelocitySensor/VelocityDipThreshold", 10);
    deccelThreshold =
        new LoggedTunableNumber(key + "/VelocitySensor/DecellerationThreshold", -11.0);
    upperBoundAccel = new LoggedTunableNumber(key + "/VelocitySensor/UpperBoundAccelRange", 10);
    lowerBoundAccel = new LoggedTunableNumber(key + "/VelocitySensor/LowerBoundAccelRange", -10);
  }

  public void update(double velocity) {
    double timeNow = Timer.getTimestamp();

    dipDetected = false;
    double acceleration = 0;
    if (peakVelocitySamples.isEmpty()) {
      acceleration = velocity;
    } else {
      acceleration = (velocity - lastVelocity) / (timeNow - timeOld);
    }

    // no longer accelarting but not at rest
    if ((withinRange(acceleration, lowerBoundAccel.getAsDouble(), upperBoundAccel.getAsDouble())
            && velocity >= 6000)
        || (peakVelocitySamples.isEmpty() && velocity >= 6000)) {
      peakVelocitySamples.add(velocity);
    }

    if (peakVelocitySamples.size() > 100) {
      peakVelocitySamples.remove(0);
      averageVelocity =
          StatUtils.mean(peakVelocitySamples.stream().mapToDouble(Double::doubleValue).toArray());
      dipDetected =
          (velocity < averageVelocity - velDipThreshold.getAsDouble())
              && acceleration < deccelThreshold.getAsDouble();
    }
    Logger.recordOutput(key + "/VelocitySensor/Acceleration", acceleration);
    Logger.recordOutput(key + "/VelocitySensor/Velocity", velocity);
    lastVelocity = velocity;
  }

  public boolean getDip() {
    return dipDetected;
  }

  // maybe move to its own thing just a lil helper function cuz i hate doing these
  private static boolean withinRange(double value, double min, double max) {
    return value >= min && value <= max;
  }
}
