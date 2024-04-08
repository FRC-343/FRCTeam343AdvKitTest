package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOSparkMax;

public class IntakeCommand extends Command {
  private final Intake intake;

  private double kIntakeSpeed;

  public IntakeCommand(double intakeSpeed) {
    intake = new Intake(new IntakeIOSparkMax());
    kIntakeSpeed = intakeSpeed;

    addRequirements(intake);
  }

  public IntakeCommand() {
    this(-10); // defaults to .8 speed
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runSpeed(kIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getNoteDetector() == false;
  }
}
