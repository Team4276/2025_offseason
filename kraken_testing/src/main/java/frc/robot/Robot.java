// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

  public Robot() {
    TalonFXConfiguration motor = new TalonFXConfiguration();
    motor.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.CurrentLimits.SupplyCurrentLimit = 40.0;
    motor.CurrentLimits.StatorCurrentLimit = 90.0;

    motor.Slot0.kP = 5.0;
    motor.Slot0.kI = 0.0;
    motor.Slot0.kD = 0.0;

    motor.Slot0.kS = 0.2;

    motor.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.MotionMagic.MotionMagicAcceleration = 150.0;
    motor.MotionMagic.MotionMagicCruiseVelocity = 150.0;
    motor.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor1.clearStickyFaults();
    motor2.clearStickyFaults();

    motor1.getConfigurator().apply(motor);
    motor2.getConfigurator().apply(motor);

    positionRot = motor1.getPosition();
    velocityRPS = motor1.getVelocity();
    voltageOut = motor1.getMotorVoltage();

    motor2.setControl(new Follower(1, true));
    SmartDashboard.putNumber("Elevator_Position_Setpoint", 0.5);
    SmartDashboard.putNumber("Elevator_Position_Setpoint_2", 10.0);
    SmartDashboard.putNumber("Elevator_Position_Setpoint_3", 20.0);
    SmartDashboard.putNumber("Elevator_Position_Setpoint_4", 40.0);
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
      motor1.setControl(new MotionMagicVoltage(SmartDashboard.getNumber("Elevator_Position_Setpoint", 0.5)));
    } else if (controller.getBButton()) {
      motor1.setControl(new MotionMagicVoltage(SmartDashboard.getNumber("Elevator_Position_Setpoint_2", 10.0)));

    } else if (controller.getXButton()) {
      motor1.setControl(new MotionMagicVoltage(SmartDashboard.getNumber("Elevator_Position_Setpoint_3", 20.0)));

    } else if (controller.getYButton()) {
      motor1.setControl(new MotionMagicVoltage(SmartDashboard.getNumber("Elevator_Position_Setpoint_4", 40.0)));

    } else {
      double scalar = 3.0;
      double output = controller.getRightY() * controller.getRightY();
      motor1.setVoltage(scalar * output * Math.signum(controller.getRightY()));
    }
  }

  @Override
  public void teleopExit() {
    motor1.setControl(new VoltageOut(0.0));
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
