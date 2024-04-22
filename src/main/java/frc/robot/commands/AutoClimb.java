package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.Climber;

public class AutoClimb extends Command {

  Climber m_Climber;

  public AutoClimb() {}

  @Override
  public void execute() {

    double speed =
        (Math.abs(m_Climber.Encoder()) + 35)
            / 100.0; // equivilent to a PID (P only), goes proportionally slower the closer you are
    speed = MathUtil.clamp(speed, .01, 1);
    m_Climber.runSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_Climber.runSpeed(0.0);
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
