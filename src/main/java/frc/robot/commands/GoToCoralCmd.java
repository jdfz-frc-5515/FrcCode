// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class GoToCoralCmd extends Command {
    private final CommandSwerveDrivetrain m_subsystem;
    private Pose2d currentCoral = new Pose2d(-1, -1, new Rotation2d(3));
    private int flew = 0;

    public GoToCoralCmd(CommandSwerveDrivetrain subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    public Pose2d getTargetPose2d() {
        String coralLimeLight = Constants.LIME_LIGHT_OBJECT_DETECTION;
        double lh = 0.922; // ll to ground height
        double la = Math.toRadians(35); // ll angle against the wall
        double tx = Math.toRadians(LimelightHelpers.getTX(coralLimeLight));
        double ty = Math.toRadians(LimelightHelpers.getTY(coralLimeLight));
        double lg1 = Math.tan(Math.PI / 2 - la + ty) * lh;
        double lg2 = Math.sqrt(Math.pow(lg1, 2) + Math.pow(lh, 2)) * Math.tan(tx);
        double da = Math.sqrt(Math.pow(lg1, 2) + Math.pow(lg2, 2)) * 0.6; // direct distance
        Pose2d robotPose = m_subsystem.getPose();
        double robotAngle = m_subsystem.getPose().getRotation().getRadians();
        double Cx = robotPose.getX() + Math.cos(robotAngle - Math.PI) * da;
        double Cy = robotPose.getY() + Math.sin(robotAngle - Math.PI) * da;
        Rotation2d Cr = new Rotation2d(robotAngle - tx * 2);
        Pose2d coralPose2d = new Pose2d(Cx, Cy, Cr);
        return coralPose2d;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        String coralLimelight = Constants.LIME_LIGHT_OBJECT_DETECTION;
        boolean hasTarget = LimelightHelpers.getTV(coralLimelight);
        SmartDashboard.putBoolean("ll hasT", hasTarget);
        int didFlyTolerance = 10;
        if (!hasTarget) {
            flew += 1;
            if (flew >= didFlyTolerance) {
                System.out.println("flewn");
                m_subsystem.customStopMoving(true);
                // find new coral here
                return;
            }
        } else {
            flew = 0;
        }
        Pose2d tempCoralPose = getTargetPose2d();
        // SmartDashboard.putNumber("coral angle",
        // tempCoralPose.getRotation().getDegrees());
        // SmartDashboard.putNumber("current angle",
        // m_subsystem.getPose().getRotation().getDegrees());
        double distanceTolerance = 0.5;
        double eucDistance = tempCoralPose.getTranslation().getDistance(currentCoral.getTranslation());
        // SmartDashboard.putNumber("current coral X", currentCoral.getX());
        // SmartDashboard.putNumber("Target Distance", eucDistance);
        if (eucDistance < distanceTolerance) {
            System.out.println("no change");
            m_subsystem.autoMoveToPose(currentCoral);
            return;
        } else {
            System.out.println("change");
            currentCoral = tempCoralPose;
        }
        m_subsystem.autoMoveToPose(currentCoral);
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
