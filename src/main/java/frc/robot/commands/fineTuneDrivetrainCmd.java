// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;

/** An example command that uses an example subsystem. */
public class fineTuneDrivetrainCmd extends Command {
    private final CommandSwerveDrivetrain m_subsystem;
    private int direction;

    // direction: 0 up 1 left 2 down 3 right
    public fineTuneDrivetrainCmd(CommandSwerveDrivetrain subsystem, int povDirection) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
        direction = povDirection;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // double x_speed = (direction == 1 || direction == 3) ? 0.5 : 0;
        // double y_speed = (direction == 0 || direction == 2) ? 0.5 : 0;

        if (direction < 4) {
            double x_speed = 0;
            double y_speed = 0;
            if (direction == 0) {
                x_speed = 0.5;
            }
            if (direction == 2) {
                x_speed = -0.5;
            }
            if (direction == 1) {
                y_speed = 0.5;
            }
            if (direction == 3) {
                y_speed = -0.5;
            }
            SmartDashboard.putBoolean("min moving", true);
            m_subsystem.customMoveWithSpeed(x_speed, y_speed);
            SmartDashboard.putBoolean("min moving", false);
        }
        else if (direction == 4) {
            // 逆时针旋转
            m_subsystem.customRotate(0.5);
        }
        else if (direction == 5) {
            // 顺时针旋转
            m_subsystem.customRotate(-0.5);
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
