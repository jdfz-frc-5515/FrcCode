// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class GoForward extends Command {
    private final CommandSwerveDrivetrain m_subsystem;
    private final UpperSystem2025Cmd m_upperCmd;

    double m_startTime = 0;
    double m_duration = 0;
    boolean m_isFinished = false;
    public GoForward(CommandSwerveDrivetrain subsystem, UpperSystem2025Cmd cmd, double duration) {
        m_subsystem = subsystem;
        addRequirements(subsystem);

        m_upperCmd = cmd;
        m_duration = duration;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
        m_isFinished = false;
    }

    // Called every time the scheduler runs while the command 0heduled.
    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() - m_startTime < m_duration) {
            m_subsystem.customMoveWithSpeed(-1, 0);

            if (m_upperCmd.getIsGroundIntakeCoralIn()) {
                m_subsystem.customStopMoving(true);
                m_isFinished = true;
            }
        }
        else {
            m_subsystem.customStopMoving(true);
            m_isFinished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.customStopMoving(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
