// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class FaceGroundCoralCmd extends Command {
    final double OFFSET = 0.;
    private final CommandSwerveDrivetrain m_subsystem;
    private Pose2d currentCoral = null;// new Pose2d(-1, -1, new Rotation2d(3));


    public FaceGroundCoralCmd(CommandSwerveDrivetrain subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    public Pose2d getTargetPose2d() {
        String coralLimeLight = Constants.LIME_LIGHT_OBJECT_DETECTION;
        // double lh = 0.83; // ll to ground height
        // double la = Math.toRadians(45); // ll angle against the wall
        double tx = Math.toRadians(LimelightHelpers.getTX(coralLimeLight));
        if (tx < 3) {
            return m_subsystem.getPose();
        }
        // double ty = Math.toRadians(LimelightHelpers.getTY(coralLimeLight));
        // double lg1 = Math.tan(la + ty) * lh;
        // double k = lh / Math.cos(la+ty);
        // double lg2 = k * Math.tan(tx);
        // double da = Math.sqrt(Math.pow(lg1, 2) + Math.pow(lg2, 2)) + OFFSET; // direct distance
        Pose2d robotPose = m_subsystem.getPose();
        double robotAngle = m_subsystem.getPose().getRotation().getRadians();
        // double Cx = robotPose.getX() + Math.cos(robotAngle - Math.PI) * da;
        // double Cy = robotPose.getY() + Math.sin(robotAngle - Math.PI) * da;
        Rotation2d Cr = new Rotation2d(robotAngle - tx * 1.2);
        Pose2d coralPose2d = new Pose2d(robotPose.getX(), robotPose.getY(), Cr);
        return coralPose2d;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentCoral = m_subsystem.getPose();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d tempCoralPose = getTargetPose2d();
        currentCoral = tempCoralPose;
        m_subsystem.autoMoveToPose(currentCoral, 1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        currentCoral = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_subsystem.isAtPose(currentCoral)) {
            return true;
        }
        return false;
    }
}
