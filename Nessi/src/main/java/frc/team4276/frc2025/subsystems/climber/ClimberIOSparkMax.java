package frc.team4276.frc2025.subsystems.climber;

import static frc.team4276.util.SparkUtil.ifOk;
import static frc.team4276.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

public class ClimberIOSparkMax implements ClimberIO {
  private SparkMax whench;
  private SparkFlex wheel;
  private final RelativeEncoder relEncoder;

  private SparkMaxConfig whenchConfig;
  private SparkFlexConfig wheelConfig;
  private boolean brakeModeEnabled = true;

  public ClimberIOSparkMax(
      int whenchID, int wheelID, int whenchCurrentLimit, int wheelCurrentLimit) {
    whenchConfig = new SparkMaxConfig();
    wheelConfig = new SparkFlexConfig();

    wheel = new SparkFlex(wheelID, MotorType.kBrushless);
    wheelConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(wheelID);
    wheelConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        wheel,
        5,
        () ->
            wheel.configure(
                wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    whench = new SparkMax(whenchID, MotorType.kBrushless);
    whenchConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(whenchCurrentLimit)
        .voltageCompensation(12.0);
    tryUntilOk(
        whench,
        5,
        () ->
            whench.configure(
                whenchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    relEncoder = whench.getEncoder();
    relEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    ifOk(whench, whench::getOutputCurrent, (value) -> inputs.whenchSupplyCurrentAmps = value);
    ifOk(
        whench,
        new DoubleSupplier[] {whench::getAppliedOutput, whench::getBusVoltage},
        (values) -> inputs.appliedVoltageWhench = values[0] * values[1]);
    ifOk(whench, whench::getMotorTemperature, (value) -> inputs.whenchTempCelsius = value);

    ifOk(wheel, wheel::getOutputCurrent, (value) -> inputs.wheelsSupplyCurrentAmps = value);
    ifOk(
        wheel,
        new DoubleSupplier[] {wheel::getAppliedOutput, wheel::getBusVoltage},
        (values) -> inputs.appliedVoltageWheels = values[0] * values[1]);
    ifOk(wheel, wheel::getMotorTemperature, (value) -> inputs.wheelsTempCelsius = value);

    ifOk(whench, relEncoder::getPosition, (value) -> inputs.position = value);
  }

  @Override
  public void runWheelsAtVolts(double volts) {
    wheel.setVoltage(volts);
  }

  @Override
  public void runRunWhenchAtVolts(double volts) {
    whench.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    if (brakeModeEnabled == false) {
      relEncoder.setPosition(0);
    }
    brakeModeEnabled = enabled;
    new Thread(
            () -> {
              tryUntilOk(
                  whench,
                  5,
                  () ->
                      whench.configure(
                          whenchConfig.idleMode(
                              brakeModeEnabled
                                  ? SparkBaseConfig.IdleMode.kBrake
                                  : SparkBaseConfig.IdleMode.kCoast),
                          SparkBase.ResetMode.kNoResetSafeParameters,
                          SparkBase.PersistMode.kNoPersistParameters));
            })
        .start();
  }
}
