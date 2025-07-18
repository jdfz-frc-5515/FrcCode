package frc.robot.commands;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;

public class MoveToCmd extends Command {
    private CommandSwerveDrivetrain m_swerve;
    private ImprovedCommandXboxController m_controller;
    private boolean isInitOk = false;

    public MoveToCmd(CommandSwerveDrivetrain swerve, ImprovedCommandXboxController controller) {
        this.m_swerve = swerve;
        addRequirements(this.m_swerve);
        m_controller = controller;
        schedule();
    }

    @Override
    public void initialize() {
        isInitOk = false;
        // System.out.println("MoveTo2025Cmd init");
        // ControlPadHelper.VirtualControl info = ControlPadHelper.getVirtualControl();
        // if (info.isTap == false || (MiscUtils.compareDouble(info.x, 0) && MiscUtils.compareDouble(info.y, 0))) {
        //     System.out.println("MoveTo2025Cmd VirtualControl is not available");
        //     return;
        // }
        // Pose2d robotPos = s_Swerve.getPose();
        // double degree = MiscUtils.calcAngle2Point(robotPos.getX(), robotPos.getY(), info.x, info.y);
        // Pose2d targetPos = new Pose2d(info.x, info.y, Rotation2d.fromDegrees(degree));

        // m_moveToSubSys.init(targetPos);
        isInitOk = true;
    }

    @Override
    public void execute() {
        
        Pose2d pos = new Pose2d(3.961, 2.782, Rotation2d.fromDegrees(60));      // aprilTag 17

        m_swerve.hybridMoveToPose(pos, m_controller, FieldConstants.reefTranslationAdjustmentRange, FieldConstants.reefRotationAdjustmentRangeDegs);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[[[[[[[[[[[[[[ END ]]]]]]]]]]]]]] :: " + interrupted);
        if (interrupted) {
            SmartDashboard.putString("MoveTo2025Cmd", "interrupted");
        }
        Pose2d pos = m_swerve.getPose();
        System.out.println("end pos: " + pos.toString());
    }

    @Override
    public boolean isFinished() {
        if (isInitOk == false) {
            return true;
        }

        // switch (this.m_moveToSubSys.getAimMoveCmdState()) {
        //     case MOVE_TO_CMD_STATE_FINISHED:
        //         System.out.println("isfinish 1");
        //         SmartDashboardEx.putString("MoveTo2025Cmd", "AIM_MOVE_CMD_STATE_FINISHED");
        //         return true;
        //     case MOVE_TO_CMD_STATE_RUNNING:
        //     // System.out.println("isfinish 4");
        //         return false;
        //     case MOVE_TO_CMD_STATE_UNKOWN:
        //         // System.out.println("isfinish 5");
        //         SmartDashboardEx.putString("MoveTo2025Cmd", "AIM_MOVE_CMD_STATE_UNKOWN");
        //         return false;
        // }
        
        // System.out.println("isfinish 6");
        // return false;

        return m_swerve.isAtTargetPose();
    }
}
