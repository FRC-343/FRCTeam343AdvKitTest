package frc.robot.subsystems.ShooterAngle;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterAngle extends SubsystemBase {
  private final ShooterAngleIO io;
  private final ShooterAngleIOInputsAutoLogged inputs = new ShooterAngleIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;
  private final DigitalInput AngleBack = new DigitalInput(5);
  private final DigitalInput AngleFront = new DigitalInput(4);

  /** Creates a new Intake. */
  public ShooterAngle(ShooterAngleIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runSpeed(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Angle", inputs);
    io.AngleBack(getAngleBack());
    io.AngleFront(getAngleFront());
  }

  public boolean getAngleBack() {
    return AngleBack.get(); // false = note in intake
  }

  public boolean getAngleFront() {
    return AngleFront.get(); // false = note in intake
  }

  /** Run open loop at the specified voltage. */
  public void runSpeed(double Speed) {
    if (getAngleBack() && Speed > 0.0) {
      io.setSpeed(0);
    } else if (getAngleFront() && Speed < 0.0) {
      io.setSpeed(0);
    } else {
      io.setSpeed(Speed);
    }
  }



  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = velocityRPM;
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log Intake setpoint
    Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
  }

  public boolean AngleBack(boolean Noted) {
    return getAngleBack();
  }

  /** Stops the Intake. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRPS);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRPS;
  }
}
