// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

import java.io.Console;

import com.ctre.phoenix6.swerve.SwerveRequest;

/** An example command that uses an example subsystem. */
public class FaceObjectCmd extends Command {
  private final CommandSwerveDrivetrain m_subsystem;
  public FaceObjectCmd(CommandSwerveDrivetrain subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV("limelight-right");
    // System.out.println(hasTarget);
    SmartDashboard.putBoolean("ll hasT", hasTarget);
    double tx = LimelightHelpers.getTX("limelight-right");
    double ty = LimelightHelpers.getTY("limelight-right");
    double ta = LimelightHelpers.getTA("limelight-right");
    if (!hasTarget) {
        tx = 0;
        ty = 0;
    }
    System.out.println(-tx*0.2);
    SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
    double rotationalRate = -tx*0.05;
    if (Math.abs(tx) < 3) {
      rotationalRate = 0;
    }
    double forwardSpeed = 0.0;
    if (0 < ta && ta < 3.5) {
        forwardSpeed = Math.abs(3.5-ta) * 0.5;
        // forwardSpeed = 0.1;
    }
    request = request.withRotationalRate(rotationalRate);
    request = request.withVelocityX(forwardSpeed);
    m_subsystem.setControl(request.withVelocityY(0).withDeadband(0.0));
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
