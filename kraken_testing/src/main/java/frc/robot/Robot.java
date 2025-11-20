// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  public Robot() {
    motor2.setControl(new Follower(1, true));
  }

  @Override
  public void robotPeriodic() {
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

  private TalonFX motor1 = new TalonFX(1);
  private TalonFX motor2 = new TalonFX(2);
  private XboxController controller = new XboxController(0);

  @Override
  public void teleopPeriodic() {
    double scalar = 3.0;
    double output = controller.getRightY() * controller.getRightY();
    motor1.setVoltage(scalar * output * Math.signum(controller.getRightY()));
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
