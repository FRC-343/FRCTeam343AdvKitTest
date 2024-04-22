package frc.robot.subsystems.ShooterAngle;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterAngleIO {
  @AutoLog
  public static class ShooterAngleIOInputs {
    public double positionRad = 0.0;
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public boolean AngleBack;
    public boolean AngleFront;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterAngleIOInputs inputs) {}

  public default void AngleFront(Boolean Front) {}

  public default void AngleBack(Boolean Back) {}

  /** Run open loop at the specified voltage. */
  public default void setSpeed(double Speed) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRPS, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
