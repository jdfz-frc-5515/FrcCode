// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.SmartDashboardEx;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ControlPadHelper;

/** An example command that uses an example subsystem. */
public class GoToCoralCmd extends Command {
    protected final CommandSwerveDrivetrain m_subsystem;
    protected Pose2d m_targetPos;

    protected boolean isInitOk = true;
    protected BooleanSupplier m_isBallModeSupplier;
    protected BooleanSupplier m_isSourceModeSupplier;
    protected boolean m_isSourceLeft = false;
    public GoToCoralCmd(CommandSwerveDrivetrain subsystem, BooleanSupplier isBallMode, BooleanSupplier isSourceMode, boolean isLeft) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
        m_isSourceModeSupplier = isSourceMode;
        m_isBallModeSupplier = isBallMode;
        m_isSourceLeft = isLeft;
    }

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
        m_targetPos = null;
        
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

        if (m_isBallModeSupplier.getAsBoolean()) {
            m_targetPos = MiscUtils.getCoralBallPos(apId);
        }
        else if (m_isSourceModeSupplier.getAsBoolean()) {
            m_targetPos = MiscUtils.getSourcePosByAlliance(m_isSourceLeft);
        }
        else if (info.branch == 1 || info.branch == -1) {
            m_targetPos = MiscUtils.getCoralShooterPos(apId, info.branch == -1);
        }
        else {
            // targetPos = new Pose2d();
        }

        // m_targetPos = targetPos;
        // for test
        if (m_targetPos == null) {
            isInitOk = false;
        }
        else {
            SmartDashboardEx.putString("MoveToCoralState", String.format("AP: %d, level: %d, branch: %d, targetPos: %s", apId, info.level, info.branch, m_targetPos.toString()));
        }
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_targetPos != null) {
            m_subsystem.autoMoveToPose(m_targetPos);
        }
        
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
