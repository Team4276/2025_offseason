package frc.team4276.frc2025.subsystems.superstructure.clopper.climber;

import static frc.team4276.frc2025.subsystems.superstructure.clopper.ClopperConstants.*;
import static frc.team4276.util.SparkUtil.ifOk;
import static frc.team4276.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.util.dashboard.LoggedTunableProfile;
import java.util.function.DoubleSupplier;

public class ClimberIOSparkMax implements ClimberIO {
  private SparkMax whench;
  private SparkFlex wheel;
  private final RelativeEncoder relEncoder;
  private final SparkClosedLoopController closedLoopController;

  private SparkMaxConfig whenchConfig;
  private SparkFlexConfig wheelConfig;
  private boolean brakeModeEnabled = true;

  private final LoggedTunableProfile profile =
      new LoggedTunableProfile("Hopper", climberMaxVel, climberMaxAccel);
  private TrapezoidProfile.State prevState = new TrapezoidProfile.State();

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
    closedLoopController = whench.getClosedLoopController();
    whenchConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(whenchCurrentLimit)
        .voltageCompensation(12.0);
    whenchConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false)
        .pidf(climberkP, 0.0, 0.0, 0.0);
    whenchConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 50))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    whenchConfig.softLimit.reverseSoftLimit(0.0);

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

    if (DriverStation.isDisabled()) {
      prevState = new TrapezoidProfile.State(inputs.position, 0.0);
    }
  }

  @Override
  public void runMotionMagikSetpoint(double setpoint) {
    prevState = profile.calculate(0.02, prevState, new TrapezoidProfile.State(setpoint, 0.0));
    closedLoopController.setReference(
        prevState.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        climberkV * prevState.velocity,
        ArbFFUnits.kVoltage);
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
