// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.utils.MiscUtils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.ControlPadHelper;

/** An example command that uses an example subsystem. */
public class GoToCoralCmd extends Command {
    private final CommandSwerveDrivetrain m_subsystem;
    private Pose2d m_targetPos;

    private boolean isInitOk = true;
    public GoToCoralCmd(CommandSwerveDrivetrain subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    // public Pose2d getTargetPose2d() {
    //     String coralLimeLight = Constants.LIME_LIGHT_OBJECT_DETECTION;
    //     double lh = 0.922; // ll to ground height
    //     double la = Math.toRadians(45); // ll angle against the wall
    //     double tx = Math.toRadians(LimelightHelpers.getTX(coralLimeLight));
    //     double ty = Math.toRadians(LimelightHelpers.getTY(coralLimeLight));
    //     double lg1 = Math.tan(Math.PI / 2 - la + ty) * lh;
    //     double lg2 = Math.sqrt(Math.pow(lg1, 2) + Math.pow(lh, 2)) * Math.tan(tx);
    //     double da = Math.sqrt(Math.pow(lg1, 2) + Math.pow(lg2, 2)); // direct distance
    //     Pose2d robotPose = m_subsystem.getPose();
    //     double robotAngle = m_subsystem.getPose().getRotation().getRadians();
    //     double shiftLeft = 0.05;
    //     double shiftAngle = robotAngle;
    //     double shiftX = Math.sin(shiftAngle)*shiftLeft;
    //     double shiftY = Math.cos(shiftAngle)*shiftLeft;
    //     double Cx = robotPose.getX() + Math.cos(robotAngle - Math.PI) * da+shiftX;
    //     double Cy = robotPose.getY() + Math.sin(robotAngle - Math.PI) * da-shiftY;
    //     Rotation2d Cr = new Rotation2d(robotAngle - tx);
    //     Pose2d coralPose2d = new Pose2d(Cx, Cy, Cr);
    //     return coralPose2d;
    // }


    public long getNearestSeenCoralAprilTag() {
        Constants.FieldInfo.APInfo[] apList = MiscUtils.getApInfoByAlliance();
        if (apList.length == 0) {
            return -1;
        }
        Pose2d robotPos = m_subsystem.getPose();
        long minDistanceApId = -1;
        double minDistance = Double.MAX_VALUE;
        for (Constants.FieldInfo.APInfo ap : apList) {
            Pose2d apPose = new Pose2d(ap.getX(), ap.getY(), new Rotation2d(Units.degreesToRadians(ap.getTheta())));

            double distance = robotPos.getTranslation().getDistance(apPose.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                minDistanceApId = (long)ap.getId();
            }
        }

        return minDistanceApId;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isInitOk = true;
         long targetApId = -1;
        if (DriverStation.isAutonomous()) {
            ControlPadHelper.ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
            targetApId = info.aprilTagId;
        }
        else {
            targetApId = getNearestSeenCoralAprilTag();
            if (targetApId == -1) {
                isInitOk = false;
                return;
            }
    
            ControlPadHelper.setControlPadInfoData(targetApId, -10l, -10l);
        }


        ControlPadHelper.ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
        if (info == null) {
            isInitOk = false;
            return;
        }
        long apId = info.aprilTagId;
        Pose2d targetPos;
        if (info.branch == 0) {
            targetPos = MiscUtils.getCoralBallPos(apId);
        }
        else if (info.branch == 1 || info.branch == -1) {
            targetPos = MiscUtils.getCoralShooterPos(apId, info.branch == -1);
        }
        else {
            targetPos = new Pose2d();
        }

        m_targetPos = targetPos;
        // for test
        SmartDashboard.putString("MoveToCoralState", String.format("AP: %d, level: %d, branch: %d, targetPos: %s", apId, info.level, info.branch, targetPos.toString()));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.autoMoveToPose(m_targetPos);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!isInitOk) {
            return true;
        }

        if (m_subsystem.isAtTargetPose()) {
            m_subsystem.customStopMoving(true);
            return true;
        }
        return false;
    }
}
