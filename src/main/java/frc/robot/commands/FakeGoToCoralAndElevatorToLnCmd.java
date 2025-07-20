package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;

public class FakeGoToCoralAndElevatorToLnCmd extends GoToCoralCmd {
    private UpperSystem2025Cmd m_upSys;
    final double LIFTING_DISTANCE = 0.5; // meters, distance to lift the robot after reaching the coral
    boolean m_isMovingDone = false;
    boolean m_isElevatorToLnDone = false;
    boolean m_isElevatorToLning = false;
    UpperSystem2025Cmd.STATE m_liftState = UpperSystem2025Cmd.STATE.NONE;

    double m_fakeStartTime = 0;

    public FakeGoToCoralAndElevatorToLnCmd(CommandSwerveDrivetrain subsystem, UpperSystem2025Cmd upSys) {
        super(subsystem, ()->false,  ()->false, true);
        m_upSys = upSys;
    }

    @Override
    public void initialize() {
        // super.initialize();
        m_isMovingDone = false;
        m_isElevatorToLnDone = false;
        m_isElevatorToLning = false;
        m_liftState = UpperSystem2025Cmd.STATE.NONE;

        m_fakeStartTime = Timer.getFPGATimestamp();
        // if (!m_upSys.getIsCarryingCoral()) {
        //     m_isElevatorToLnDone = true;
        // }
    }

    @Override
    public void execute() {
        // super.execute(); 
        // if (!m_isMovingDone && m_subsystem.isAtTargetPose()) {
        //     m_isMovingDone = true;
        //     m_subsystem.customStopMoving(true);
        // }

        if (!m_isMovingDone) {
            double now = Timer.getFPGATimestamp();
            if (now - m_fakeStartTime >= 2) {
                m_isMovingDone = true;
            }
        }

        if (!m_isElevatorToLnDone) {
            if (m_isElevatorToLning) {
                if (m_upSys.isStateDone(m_liftState)) {
                    m_isElevatorToLnDone = true;
                    m_isElevatorToLning = false;
                    m_liftState = UpperSystem2025Cmd.STATE.NONE;
                }
            }
            else if (!m_isElevatorToLning) {
                m_liftState = m_upSys.setStateLn();
                if (m_liftState != UpperSystem2025Cmd.STATE.NONE) {
                    m_isElevatorToLning = true;
                }
            }            
        }

    }

    @Override
    public boolean isFinished() {
        return m_isMovingDone && m_isElevatorToLnDone;
    }
}
