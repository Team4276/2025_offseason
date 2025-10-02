package frc.team4276.util.ios;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.team4276.frc2025.subsystems.superstructure.drive.SparkOdometryThread;
import java.util.Queue;

public class GyroIOADIS implements GyroIO {
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  // private double timeSinceLastReset = 0;
  // private int iter = 0;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOADIS() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(gyro::getAngle);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();

    inputs.yawPosition = Rotation2d.fromDegrees(gyro.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRate());

    /*
     * if (timeSinceLastReset > 60 && Math.hypot(inputs.pitchPosition,
     * inputs.rollPosition) < 2) {
     * gyro.setGyroAngle(gyro.getPitchAxis(), 0);
     * gyro.setGyroAngle(gyro.getRollAxis(), 0);
     * timeSinceLastReset = 0;
     * iter++;
     * } else {
     * timeSinceLastReset = (Timer.getFPGATimestamp() - (60 * iter));
     * }
     */
    // inputs.pitchPosition = gyro.getAngle(gyro.getPitchAxis());
    // inputs.rollPosition = gyro.getAngle(gyro.getRollAxis());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void calibrate() {
    // gyro.configCalTime(CalibrationTime._1s);
    // gyro.calibrate();

    // try {
    // Thread.sleep(260);

    // } catch (Exception e) {
    // System.err.println(e.toString());
    // System.exit(1);
    // }
  }
}
