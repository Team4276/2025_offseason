package frc.team4276.util.drivers;

import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.stat.StatUtils;
import org.littletonrobotics.junction.Logger;

public class CurrentSensor {
  private final String key;

  private final LoggedTunableNumber spikeThreshold;
  private final LoggedTunableNumber outlierThreshold;
  private List<Double> currentSamples = new ArrayList<>();
  private double currentAverage;

  boolean spikeDetected;

  public CurrentSensor(String key) {
    this.key = key;

    spikeThreshold = new LoggedTunableNumber(key + "/CurrentSensor/significantCurrentSpike", 10);
    outlierThreshold = new LoggedTunableNumber(key + "/CurrentSensor/currentOutlierValue", 45);
  }

  public void update(double current, boolean isIdle) {
    spikeDetected = false;
    Logger.recordOutput(key + "/CurrentSensor/current", current);

    if (current < outlierThreshold.getAsDouble() && !isIdle) {
      currentSamples.add(current);
    }

    if (currentSamples.size() > 100) {
      currentSamples.remove(0);
      currentAverage =
          StatUtils.mean(currentSamples.stream().mapToDouble(Double::doubleValue).toArray());
      spikeDetected = current > currentAverage + spikeThreshold.getAsDouble();
    } else {
      spikeDetected = false;
    }
  }

  public boolean getDetection() {
    return spikeDetected;
  }
}
