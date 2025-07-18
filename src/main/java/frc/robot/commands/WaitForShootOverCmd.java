package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake2025.Intake2025;

public class WaitForShootOverCmd extends Command {
    private Intake2025 m_intake;


    public WaitForShootOverCmd(Intake2025 intake) {
        this.m_intake = intake;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_intake.getIsCarryingCoral() == false;
    }
}
