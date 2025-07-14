// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PlayCandleSubsystem;

/** An example command that uses an example subsystem. */
public class CandleCmd extends Command {
  private final PlayCandleSubsystem m_candleSubsystem;
  private boolean nextAnime;
  public CandleCmd(PlayCandleSubsystem subsystem, boolean nextAnime) {
    m_candleSubsystem = subsystem;
    this.nextAnime = nextAnime;
    addRequirements(subsystem);
  }
  @Override
  public void initialize() {
    m_candleSubsystem.initCandle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (nextAnime) {
      m_candleSubsystem.nextAnim();
    }
    else {
      m_candleSubsystem.previousAnim();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
