package frc.team4276.frc2025.subsystems.elevator;

import static frc.team4276.frc2025.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.team4276.frc2025.Ports;
import frc.team4276.lib.ctre.TalonFXFactory;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX master;
  private final TalonFX follower;

  VoltageOut voltageOut = new VoltageOut(0.0);;
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

  private final StatusSignal<Angle> elevatorPosition;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Voltage> elevatorAppliedVolts;
  private final StatusSignal<Current> elevatorSupplyCurrentAmps;
  private final StatusSignal<Current> elevatorStatorCurrentAmps;
  private final StatusSignal<Temperature> elevatorMotor1Temp;
  private final StatusSignal<Temperature> elevatorMotor2Temp;

  public ElevatorIOTalonFX() {
    master = TalonFXFactory.createDefaultTalon(Ports.ELEVATOR_LEADER);
    follower = TalonFXFactory.createDefaultTalon(Ports.ELEVATOR_FOLLOWER);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120.0;

    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;

    motorConfig.Slot0.kS = kS;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotionMagic.MotionMagicAcceleration = maxAccelRpss;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelRps;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    master.getConfigurator().apply(motorConfig);
    follower.getConfigurator().apply(motorConfig);

    elevatorPosition = master.getPosition();
    elevatorVelocity = master.getRotorVelocity();
    elevatorAppliedVolts = master.getMotorVoltage();
    elevatorSupplyCurrentAmps = master.getSupplyCurrent();
    elevatorStatorCurrentAmps = master.getStatorCurrent();
    elevatorMotor1Temp = master.getDeviceTemp();
    elevatorMotor2Temp = follower.getDeviceTemp();

    follower.setControl(new Follower(Ports.ELEVATOR_LEADER, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        elevatorPosition,
        elevatorVelocity,
        elevatorAppliedVolts,
        elevatorSupplyCurrentAmps,
        elevatorStatorCurrentAmps,
        elevatorMotor1Temp,
        elevatorMotor2Temp);

    inputs.position = elevatorPosition.getValueAsDouble() / motorRotationsPerMetre;
    inputs.velocity = elevatorVelocity.getValueAsDouble() / motorRotationsPerMetre;
    
    inputs.isConnected[0] = master.isConnected();
    inputs.appliedVolts = elevatorAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = elevatorSupplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = elevatorStatorCurrentAmps.getValueAsDouble();
    inputs.tempCelcius[0] = elevatorMotor1Temp.getValueAsDouble();

    inputs.isConnected[1] = follower.isConnected();
    inputs.tempCelcius[1] = elevatorMotor2Temp.getValueAsDouble();
  }

  /** Run to setpoint */
  @Override
  public void runSetpoint(double positionInMetres) {
    master.setControl(motionMagicVoltage.withPosition(positionInMetres * motorRotationsPerMetre));
  }

  /** Run motors at volts */
  @Override
  public void runVolts(double volts) {
    master.setControl(voltageOut.withOutput(volts));
  }

  /** Only call when disabled */
  @Override
  public void setPosition(double positionMetres) {
    master.setPosition(positionMetres * motorRotationsPerMetre);
  }

  @Override
  public void stop() {
    master.stopMotor();
    follower.stopMotor();
  }

  /** Only call when disabled */
  @Override
  public void setBrakeMode(boolean enabled) {
    master.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
