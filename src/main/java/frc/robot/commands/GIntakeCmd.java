// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntakeSubsystem;

/** An example command that uses an example subsystem. */
public class GIntakeCmd extends Command {
    private final GroundIntakeSubsystem g_subsystem;
    private boolean expand;

    public GIntakeCmd(GroundIntakeSubsystem subsystem, boolean expand) {
        g_subsystem = subsystem;
        this.expand = expand;
        addRequirements(g_subsystem);
    }
    // private final ExampleSubsystem m_subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    // public ExampleCommand(ExampleSubsystem subsystem) {
    // m_subsystem = subsystem;
    // // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
    // }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        g_subsystem.configGroundIntake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(expand);
        if (expand) {
            g_subsystem.startIntake();
            System.out.println("expand");
            g_subsystem.expandGIntake();
        } else {
            g_subsystem.reverseIntake();
            g_subsystem.retractGIntake();
            g_subsystem.stopIntake();
            // g_subsystem.zeroCC();
            System.out.println("retract");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
