// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private TalonFX motor1 = new TalonFX(1);
  private TalonFX motor2 = new TalonFX(2);
  private XboxController controller = new XboxController(0);

  private StatusSignal<Angle> positionRot;
  private StatusSignal<AngularVelocity> velocityRPS;
  private StatusSignal<Voltage> voltageOut;

  private VoltageOut motor1VoltageOut = new VoltageOut(0.0);
  private MotionMagicVoltage motor1MotionMagicVoltage = new MotionMagicVoltage(0.0);
  private VoltageOut motor2VoltageOut = new VoltageOut(0.0);

  public Robot() {
    TalonFXConfiguration motor1Config = new TalonFXConfiguration();
    motor1Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor1Config.CurrentLimits.SupplyCurrentLimit = 40.0;
    motor1Config.CurrentLimits.StatorCurrentLimit = 90.0;

    motor1Config.Slot0.kP = 0.5;
    motor1Config.Slot0.kI = 0.0;
    motor1Config.Slot0.kD = 0.0;

    motor1Config.Slot0.kS = 0.0;

    motor1Config.Feedback.RotorToSensorRatio = 1.0;

    motor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor1Config.MotionMagic.MotionMagicAcceleration = 15.0;
    motor1Config.MotionMagic.MotionMagicCruiseVelocity = 15.0;
    motor1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXConfiguration motor2Config = new TalonFXConfiguration();
    motor2Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor2Config.CurrentLimits.SupplyCurrentLimit = 30.0;
    motor2Config.CurrentLimits.StatorCurrentLimit = 60.0;

    motor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor1.clearStickyFaults();
    motor2.clearStickyFaults();

    motor1.getConfigurator().apply(motor1Config);
    motor2.getConfigurator().apply(motor2Config);

    positionRot = motor1.getPosition();
    velocityRPS = motor1.getVelocity();
    voltageOut = motor1.getMotorVoltage();

    SmartDashboard.putNumber("Elevator_Position_Setpoint", 0.0);
    SmartDashboard.putNumber("Elevator_Position_Setpoint_2", 15.0);
    SmartDashboard.putNumber("Elevator_Position_Setpoint_3", 15.0);
    SmartDashboard.putNumber("Elevator_Position_Setpoint_4", 15.0);
    SmartDashboard.putNumber("Intake_OuttakeVoltage", 10.0);
    SmartDashboard.putNumber("Intake_IntakeVoltage", -10.0);

    motor1.setPosition(0.0);
  }

  @Override
  public void robotPeriodic() {
    StatusSignal.refreshAll(positionRot, velocityRPS, voltageOut);
    SmartDashboard.putNumber("Motor Position Rotations", positionRot.getValueAsDouble());
    SmartDashboard.putNumber("Motor Velocity Rotations", velocityRPS.getValueAsDouble());
    SmartDashboard.putNumber("Motor Voltage", voltageOut.getValueAsDouble());
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if (controller.getAButton()) {
      motor1.setControl(new MotionMagicVoltage(SmartDashboard.getNumber("Elevator_Position_Setpoint", 0.0)));
    } else if (controller.getBButton()) {
      motor1.setControl(new MotionMagicVoltage(SmartDashboard.getNumber("Elevator_Position_Setpoint_2", 15.0)));

    } else if (controller.getXButton()) {
      motor1.setControl(new MotionMagicVoltage(SmartDashboard.getNumber("Elevator_Position_Setpoint_3", 15.0)));

    } else if (controller.getYButton()) {
      motor1.setControl(new MotionMagicVoltage(SmartDashboard.getNumber("Elevator_Position_Setpoint_4", 15.0)));

    } else {
      double scalar = 5.0;
      double output = controller.getRightY() * controller.getRightY();
      motor1.setControl(motor1VoltageOut.withOutput(scalar * output * Math.signum(controller.getRightY())));
    }

    if (controller.getLeftTriggerAxis() > 0.1) {
      motor2.setControl(motor2VoltageOut.withOutput(SmartDashboard.getNumber("Intake_OuttakeVoltage", 10.0)));
    } else if (controller.getRightTriggerAxis() > 0.1) {
      motor2.setControl(motor2VoltageOut.withOutput(SmartDashboard.getNumber("Intake_IntakeVoltage", -10.0)));
    } else {
      motor2.setControl(motor2VoltageOut.withOutput(0.0));
    }
  }

  @Override
  public void teleopExit() {
    motor1.setControl(motor2VoltageOut.withOutput(0.0));
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
