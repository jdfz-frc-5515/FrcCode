// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
    public static Robot inst = null;

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
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
    LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT, 2);
    LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT, 2);
    LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_OBJECT_DETECTION, 2);
    System.out.println("mimim");
  }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
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
    String coralLimelight = "limelight-right";
    LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT, 0);
    LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT, 0);
    LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_OBJECT_DETECTION, 0);
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
