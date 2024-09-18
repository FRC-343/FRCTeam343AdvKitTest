/*This File is the IO(basically setting up the motor) for our climber */

package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkRelativeEncoder;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ClimberIOSparkMax implements ClimberIO {
  private static final double GEAR_RATIO = 1;

  private final CANSparkMax leader = new CANSparkMax(16, MotorType.kBrushless);
  private final SparkPIDController pid = leader.getPIDController();
  private final RelativeEncoder lEncoder =
      leader.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

  public ClimberIOSparkMax() {
    leader.restoreFactoryDefaults();

    leader.setCANTimeout(250);

    leader.setInverted(false);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    leader.burnFlash();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
    inputs.ClimberEncoder = lEncoder.getPosition();
  }

  @Override
  public void setSpeed(double Speed) {
    leader.set(Speed);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(velocityRadPerSec, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void resetEnc() {
    lEncoder.setPosition(0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
