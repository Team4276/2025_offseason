package frc.team4276.util.hid;

import edu.wpi.first.wpilibj.Joystick;

public class CowsController {
  private double JOYSTICK_DEADBAND = 0.1;

  private final Joystick leftStick;
  private final Joystick rightStick;

  public CowsController(int left, int right) {
    leftStick = new Joystick(left);
    rightStick = new Joystick(right);
  }

  public CowsController withDeadband(double deadband) {
    this.JOYSTICK_DEADBAND = deadband;

    return this;
  }

  public Joystick getRightStick() {
    return rightStick;
  }

  public Joystick getLeftStick() {
    return leftStick;
  }

  public class JoystickOutput {
    public final double x;
    public final double y;

    public JoystickOutput(double x, double y) {
      this.x = x;
      this.y = y;
    }

    public JoystickOutput() {
      this.x = 0.0;
      this.y = 0.0;
    }
  }

  public JoystickOutput getRightWithDeadband() {
    return Math.hypot(rightStick.getX(), rightStick.getY()) < JOYSTICK_DEADBAND
        ? new JoystickOutput()
        : getRight();
  }

  /** No Deadband */
  public JoystickOutput getRight() {
    return new JoystickOutput(rightStick.getX(), rightStick.getY());
  }

  public JoystickOutput getLeftWithDeadband() {
    return Math.hypot(leftStick.getX(), leftStick.getY()) < JOYSTICK_DEADBAND
        ? new JoystickOutput()
        : getLeft();
  }

  public JoystickOutput getLeft() {
    return new JoystickOutput(leftStick.getX(), leftStick.getY());
  }

  public boolean getLT() {
    return leftStick.getTrigger();
  }

  public boolean getRT() {
    return rightStick.getTrigger();
  }
}
