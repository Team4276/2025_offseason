package frc.team4276.frc2025.subsystems.intake;

import static frc.team4276.frc2025.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.team4276.frc2025.Ports;
import frc.team4276.lib.TalonFXFactory;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX pivot;
  private final TalonFX rollers;
  private final CANcoder encoder;

  VoltageOut pivotVoltageOut = new VoltageOut(0.0);
  VoltageOut rollersVoltageOut = new VoltageOut(0.0);
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

  private final StatusSignal<Angle> pivotAngle;
  private final StatusSignal<AngularVelocity> pivotAngularVelocityRadPerSec;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotSupplyCurrentAmps;
  private final StatusSignal<Current> pivotTorqueCurrentAmps;
  private final StatusSignal<Temperature> pivotMotorTemp;

  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerMotorTemp;

  public IntakeIOTalonFX() {
    pivot = TalonFXFactory.createDefaultTalon(Ports.INTAKE_PIVOT);
    rollers = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ROLLERS);
    encoder = new CANcoder(Ports.INTAKE_CANCODER);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    pivotConfig.Slot0.kP = kP;
    pivotConfig.Slot0.kI = kI;
    pivotConfig.Slot0.kD = kD;

    pivotConfig.Slot0.kS = kS;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotionMagic.MotionMagicAcceleration = maxAccel;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVel;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = encoderOffset;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    pivot.getConfigurator().apply(pivotConfig);
    rollers.getConfigurator().apply(rollerConfig);
    encoder.getConfigurator().apply(encoderConfig);

    pivotAngle = pivot.getPosition();
    pivotAngularVelocityRadPerSec = pivot.getRotorVelocity();
    pivotAppliedVolts = pivot.getMotorVoltage();
    pivotSupplyCurrentAmps = pivot.getSupplyCurrent();
    pivotTorqueCurrentAmps = pivot.getTorqueCurrent();
    pivotMotorTemp = pivot.getDeviceTemp();

    rollerAppliedVolts = pivot.getMotorVoltage();
    rollerSupplyCurrentAmps = pivot.getSupplyCurrent();
    rollerTorqueCurrentAmps = pivot.getTorqueCurrent();
    rollerMotorTemp = pivot.getDeviceTemp();

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotAngle,
        pivotAngularVelocityRadPerSec,
        pivotAppliedVolts,
        pivotSupplyCurrentAmps,
        pivotTorqueCurrentAmps,
        pivotMotorTemp,
        rollerAppliedVolts,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerMotorTemp);

    inputs.position = pivotAngle.getValueAsDouble();
    inputs.velocity = pivotAngularVelocityRadPerSec.getValueAsDouble();
    inputs.appliedVolts[0] = pivotAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps[0] = pivotSupplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps[0] = pivotTorqueCurrentAmps.getValueAsDouble();
    inputs.tempCelcius[0] = pivotMotorTemp.getValueAsDouble();

    inputs.appliedVolts[1] = rollerAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps[1] = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps[1] = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.tempCelcius[1] = rollerMotorTemp.getValueAsDouble();
  }

  @Override
  public void runPivotVolts(double volts) {
    pivot.setControl(pivotVoltageOut.withOutput(volts));
  }

  @Override
  public void runRollerVolts(double volts) {
    rollers.setControl(rollersVoltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    pivot.stopMotor();
    rollers.stopMotor();
  }

  @Override
  public void runPivotSetpoint(double position) {
    pivot.setControl(motionMagicVoltage.withPosition(position));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    pivot.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

}
