// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.swerve.SwerveRequest;

/** An example command that uses an example subsystem. */
public class GoToCoralCmd extends Command {
  private final CommandSwerveDrivetrain m_subsystem;
  public GoToCoralCmd(CommandSwerveDrivetrain subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String coralLimelight = "limelight-right";
    double h_l = 0.19199;
    double h_3 = 0.25961;
    boolean hasTarget = LimelightHelpers.getTV(coralLimelight);
    SmartDashboard.putBoolean("ll hasT", hasTarget);
    if (!hasTarget) {
      m_subsystem.customStopMoving(true);
      return;
    }
    double txDeadBand = 3;
    double tx = LimelightHelpers.getTX(coralLimelight);
    if (Math.abs(tx) > txDeadBand) {
      m_subsystem.customRotate(-tx*0.1);
      System.out.println("mimimimi");
      return;
    }
    double ty = LimelightHelpers.getTY(coralLimelight);
    double h2 = Math.sqrt(Math.pow(h_l/Math.sin(-Math.toRadians(ty)), 2) - Math.pow(h_l, 2))+0.3;
    System.out.println(h2);
    Pose2d robotPose = m_subsystem.getPose();
    double coralX = robotPose.getX() + Math.cos(Math.toRadians(tx)) * (h2+h_3);
    double coralY = robotPose.getY() + Math.sin(Math.toRadians(tx)) * (h2+h_3);
    Pose2d coralPose = new Pose2d(coralX, coralY, robotPose.getRotation());
    // Pose2d coralPose = new Pose2d(0.5, 0.5, robotPose.getRotation());
    m_subsystem.autoMoveToPose(coralPose);
    // System.out.println(rotate);
    // System.out.println(-tx*0.2);
    // SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
    // double rotationalRate = -tx*0.05;
    // if (Math.abs(tx) < 3) {
    //   rotationalRate = 0;
    // }
    // double forwardSpeed = 0.0;
    // if (0 < ta && ta < 3.5) {
    //     forwardSpeed = Math.abs(3.5-ta) * 0.5;
    //     // forwardSpeed = 0.1;
    // }
    // request = request.withRotationalRate(rotationalRate);
    // request = request.withVelocityX(forwardSpeed);
    // m_subsystem.setControl(request.withVelocityY(0).withDeadband(0.0));
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
