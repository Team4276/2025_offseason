package frc.team4276.util.drivers;

public class ObjectSensor {
  private final CurrentSensor cSensor;
  private final VelocitySensor vSensor;

  boolean objectDetected;

  public ObjectSensor(String key) {
    cSensor = new CurrentSensor(key + "/ObjectSensor");
    vSensor = new VelocitySensor(key + "/ObjectSensor");
  }

  public void update(double current, double velocity, boolean isIdle) {
    cSensor.update(current, isIdle);
    vSensor.update(velocity);
    objectDetected = cSensor.getDetection() && vSensor.getDip();
  }

  public boolean getDetection() {
    return objectDetected;
  }
}
