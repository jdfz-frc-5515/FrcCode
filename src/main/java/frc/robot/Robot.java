// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.MiscUtils;

public class Robot extends TimedRobot {
    public static Robot inst = null;

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        // Pose2d p; 
        // p = MiscUtils.getSourcePos(1);
        // System.out.println("Source 1 Position: " + p.getTranslation().getX() + ", " + p.getTranslation().getY() + ", " + p.getRotation().getDegrees());
        // p = MiscUtils.getSourcePos(2);
        // System.out.println("Source 2 Position: " + p.getTranslation().getX() + ", " + p.getTranslation().getY() + ", " + p.getRotation().getDegrees());
        // p = MiscUtils.getSourcePos(12);
        // System.out.println("Source 12 Position: " + p.getTranslation().getX() + ", " + p.getTranslation().getY() + ", " + p.getRotation().getDegrees());
        // p = MiscUtils.getSourcePos(13);
        // System.out.println("Source 13 Position: " + p.getTranslation().getX() + ", " + p.getTranslation().getY() + ", " + p.getRotation().getDegrees());
        
        inst = this;
        Constants.initializeConstants();
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.updateAlways();
    }

    @Override
    public void disabledInit() {


        m_robotContainer.onDisabled();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.autoInit();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        m_robotContainer.update();
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.telInit();
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.update();
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.testInit();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
