package frc.team4276.frc2025.subsystems.superstructure.clopper.hopper;

import static frc.team4276.frc2025.subsystems.superstructure.clopper.ClopperConstants.*;
import static frc.team4276.util.SparkUtil.ifOk;
import static frc.team4276.util.SparkUtil.sparkStickyFault;
import static frc.team4276.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.util.dashboard.LoggedTunableProfile;
import java.util.function.DoubleSupplier;

public class HopperIOSparkMax implements HopperIO {
  private final SparkBase spark;
  private final RelativeEncoder relativeEncoder;
  private final SparkClosedLoopController closedLoopController;
  private final SparkMaxConfig config;

  // Connection debouncers
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private boolean brakeModeEnabled = true;

  private final LoggedTunableProfile profile = new LoggedTunableProfile("Hopper", 25.0, 25.0);
  private TrapezoidProfile.State prevState = new TrapezoidProfile.State();

  public HopperIOSparkMax(int id, boolean inverted) {
    spark = new SparkMax(id, MotorType.kBrushless);
    relativeEncoder = spark.getEncoder();
    closedLoopController = spark.getClosedLoopController();

    config = new SparkMaxConfig();

    config
        .inverted(inverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false)
        .pidf(kp, ki, kd, 0.0);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / readFreq))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    // Configure motor
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    // Update inputs
    sparkStickyFault = false;
    ifOk(spark, relativeEncoder::getPosition, (value) -> inputs.position = value);
    ifOk(spark, relativeEncoder::getVelocity, (value) -> inputs.velocity = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts[0] = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps[0] = value);
    ifOk(spark, spark::getMotorTemperature, (value) -> inputs.tempCelcius[0] = value);

    inputs.isCoast = !brakeModeEnabled;
    inputs.leaderMotorConnected = connectedDebounce.calculate(!sparkStickyFault);

    if (DriverStation.isDisabled()) {
      prevState = new TrapezoidProfile.State(inputs.position, 0.0);
    }
  }

  @Override
  public void runSetpoint(double setpoint, double ff) {
    closedLoopController.setReference(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
  }

  @Override
  public void runSetpoint(double setpoint) {
    runSetpoint(setpoint, 0.0);
  }

  @Override
  public void runMotionMagikSetpoint(double setpoint, double ff) {
    prevState = profile.calculate(0.02, prevState, new TrapezoidProfile.State(setpoint, 0.0));
    closedLoopController.setReference(
        prevState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
  }

  @Override
  public void runVolts(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void runCurrent(double amps) {}

  @Override
  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    new Thread(
            () -> {
              tryUntilOk(
                  spark,
                  5,
                  () ->
                      spark.configure(
                          config.idleMode(
                              brakeModeEnabled
                                  ? SparkBaseConfig.IdleMode.kBrake
                                  : SparkBaseConfig.IdleMode.kCoast),
                          SparkBase.ResetMode.kNoResetSafeParameters,
                          SparkBase.PersistMode.kNoPersistParameters));
            })
        .start();
  }

  @Override
  public void setPID(double p, double i, double d) {}

  @Override
  public void stop() {
    spark.stopMotor();
  }
}
