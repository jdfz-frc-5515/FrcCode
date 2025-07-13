package frc.robot.commands;


import java.awt.Cursor;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ControlPadHelper;
import frc.robot.Robot;
import frc.robot.ControlPadHelper.ControlPadInfo;
import frc.robot.GlobalConfig;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.GI_STATE;
import frc.robot.subsystems.Candle2025.Candle2025;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator2025.Elevator2025;
import frc.robot.subsystems.Elevator2025.Elevator2025.EV_STATE;
import frc.robot.subsystems.Intake2025.Intake2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025.TA_STATE;

import frc.robot.utils.MiscUtils;
import frc.robot.utils.ActionRunner;

public class UpperSystem2025Cmd extends Command {
    public static UpperSystem2025Cmd inst = null;

    private enum STATE {
        NONE,
        ZERO,
        READY_FOR_LOAD_GROUND_CORAL,
        READY_FOR_LOAD_UP_CORAL,
        L1,
        L2,
        L3,
        L4,
        BALL1,      // equal to READY_FOR_LOAD_BALL
        BALL2,
    }

    // -1 down, 1 up, 0 unknow
    // private int[][] table = new int[][]{
    //     //          NONE, ZERO, READY_FOR_LOAD_GROUND_CORAL  READY_FOR_LOAD_UP_CORAL, L1, L2, L3, L4, BALL1, BALL2
    //     new int[]{  0,    0,    0,                               0,                    0,  0,  0,  0,   0,     0     },// NONE
    //     new int[]{  0,    0,   -1,                              -1,                   -1, -1, -1, -1, -1,    -1     },// ZERO
    //     new int[]{  0,    1,    0,                               0,                   -1, -1, -1, -1, -1,    -1     },//READY_FOR_LOAD_GROUND_CORAL
    //     new int[]{  0,    1,    0,                               0,                    -1, -1, -1, -1, -1,    -1     },// READY_FOR_LOAD_UP_CORAL
    //     new int[]{  0,    1,    1,                               1,                    0,  -1, -1, -1, -1,    0     },// L1
    //     new int[]{  0,    1,    1,                               1,                    1,  0,  -1, -1, -1,    0     },// L2
    //     new int[]{  0,    1,    1,                               1,                    1,  1,  0,  -1, -1,    0     },// L3
    //     new int[]{  0,    1,    1,                               1,                    1,  1,  1,  0,  -1,    0     },// L4
    //     new int[]{  0,    0,    1,                               1,                    0,  0,  0,  0,   0,    1    },// BALL1
    //     new int[]{  0,    0,    1,                               1,                    0,  0,  0,  0,  -1 ,    0    },// BALL2
    // };

    // private STATE[] STATE_UP_DIR = new STATE[] {
    //     STATE.ZERO,
    //     STATE.READY_FOR_LOAD_CORAL,
    //     STATE.L1,
    //     STATE.L2,
    //     STATE.L3,
    //     STATE.L4,
    // };
    

    // private STATE[] getStatePath(STATE from, STATE to) {
    //     int fromIndex = from.ordinal();
    //     int toIndex = to.ordinal();
    //     int dir = table[toIndex][fromIndex];
    //     if (dir == 0) {
    //         return new STATE[]{};
    //     }
    //     List<STATE> path = new ArrayList<STATE>();
    //     if (dir == 1) {
    //         for (int i = fromIndex + 1; i <= toIndex; i++) {
    //             path.add(STATE_UP_DIR[i]);
    //         }
    //     }
    //     else if (dir == -1) {
    //         for (int i = fromIndex - 1; i >= toIndex; i--) {
    //             path.add(STATE_UP_DIR[i]);
    //         }
    //     }
    //     return path.toArray(new STATE[0]);
    // }

    private enum RUNNING_STATE {
        NEW_SET,
        RUNNING,
        DONE,
    }

    private STATE lastState = STATE.NONE;
    private STATE curState = STATE.ZERO;
    private RUNNING_STATE curRunningState = RUNNING_STATE.DONE;

    private CommandSwerveDrivetrain m_chassis;
    private TurningArm2025 m_turningArm;
    private Elevator2025 m_elevator;
    private Intake2025 m_intake;
    private GroundIntakeSubsystem m_groundIntake;
    private Candle2025 m_candle;
    // private MoveTo2025 m_moveTo;
    // TODO: additional subsystems: sensor for checking if we are carrying Coral;
    // outtake coral; intake coral, intake ball, outtake ball

    private Trigger test_armBtn; // test only
    private Trigger test_zeroBtn; // test only

    private Trigger resetCanCodePositionBtn;
    private Trigger resetToZeroPosBtn;
    // private Trigger switchCoralnBallBtn;
    private Trigger aimLeftCoralBtn;
    private Trigger aimRigthCoralBtn;
    private Trigger aimGroundCoralBtn;
    private Trigger intakeTrigger;
    private Trigger groundIntakeSwitchTrigger;

    private Trigger elevatorTuningUpTrigger;
    private Trigger elevatorTuningDownTrigger;
    private Trigger armTuningUpTrigger;
    private Trigger armTuningDownTrigger;


    private Trigger switchIntakeSourceTrigger;  // 切换Intake来源，地吸或者上面漏斗
    private Trigger switchCoralAndBallTrigger;  // 切换Coral模式还是Aglea模式

    private Trigger elevatorLevelUpTrigger;
    private Trigger elevatorLevelDownTrigger;

    // L1,L2,L3,L4, 有Coral时改变电梯位置，无Coral时预约下次电梯高度
    private Trigger L1Trigger;
    private Trigger L2Trigger;
    private Trigger L3Trigger;
    private Trigger L4Trigger;

    private Trigger groundIntakeOutTakeTrigger;  // 地吸反转，吐Coral


    private final boolean isDebugEnabled = true;

    private boolean isCarryingCoralFromDebug = false;
    private boolean isCarryingBallFromDebug = false;

    private Command autoMoveCmd = null;

    ActionRunner m_actionRunner = null;

    private boolean m_isIntakeFromGround = true; // 从地吸拿Coral还是从漏斗拿

    public UpperSystem2025Cmd(
            CommandSwerveDrivetrain chassis,
            TurningArm2025 turningArm, Elevator2025 elev, Intake2025 intake, GroundIntakeSubsystem groundIntake, Candle2025 candle/*  MoveTo2025 moveto,*/
        ) {

        inst = this;

        m_chassis = chassis; // 底盘不用addRequirements，这里只是引用数据，控制

        this.m_turningArm = turningArm;
        addRequirements(m_turningArm);

        this.m_elevator = elev;
        addRequirements(m_elevator);

        this.m_intake = intake;
        addRequirements(m_intake);

        this.m_groundIntake = groundIntake;
        addRequirements(m_groundIntake);

        this.m_candle = candle;
        addRequirements(m_candle);

        // this.m_moveTo = moveto;
        // addRequirements(m_moveTo);

        groundIntake.setUpperIntakeControl(()->{
            m_intake.startIntake();
        }, ()-> {});

        schedule();

        initDebug();
    }

    public void setResetToZeroPosTrigger(Trigger t) {
        if (t == null) {
            return;
        }
        if (this.resetToZeroPosBtn != null) {
            DriverStation.reportError("setResetToZeroPosTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }

        this.resetToZeroPosBtn = t;
        this.resetToZeroPosBtn.onTrue(new InstantCommand(() -> {
            if (GlobalConfig.devMode) {
                setState(STATE.ZERO);
            }
        }));
    }

    public Command getMoveToCoralCmdGroup(long aprilTagId, long level, long branch) {
        // 这是一个Command的组合，组合了移动到目标，然后抬升电梯两个动作
        Command ret = new SequentialCommandGroup(
            new InstantCommand(() -> {
                ControlPadHelper.setControlPadInfoData(aprilTagId, level, branch);
            }),
            new GoToCoralCmd(m_chassis)
            // , new InstantCommand(() -> {
            //     setStateLn();
            // })
            ,new FunctionalCommand(
                ()->{
                     //init
                     setStateLn();
                },
                ()->{
                    // onExecute
                },
                (Boolean b)->{
                    // onEnd
                },
                ()->{
                    // isFinished
                    if (curRunningState == RUNNING_STATE.DONE) {
                        return true;
                    }
                    return false;
                }
            )
        );
        System.out.println(ret.getRequirements().toString());
        return ret;
    }

    public void setAimLeftCoralTrigger(Trigger t) {
        if (t == null) {
            return;
        }
        if (this.aimLeftCoralBtn != null) {
            DriverStation.reportError("setAimLeftCoralTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }
        this.aimLeftCoralBtn = t;

        this.aimLeftCoralBtn.whileTrue(getMoveToCoralCmdGroup(-1, -1, -1));
    }

    public void setAimRightCoralTrigger(Trigger t) {
        if (t == null) {
            return;
        }
        if (this.aimRigthCoralBtn != null) {
            DriverStation.reportError("setAimRightCoralTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }
        this.aimRigthCoralBtn = t;
        this.aimRigthCoralBtn.whileTrue(getMoveToCoralCmdGroup(-1, -1, 1));
    }

    public void setAimGroundCoralTrigger(Trigger t) {
        if (t == null) {
            return;
        }
        if (this.aimGroundCoralBtn != null) {
            DriverStation.reportError("setAimGroundCoralTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }
        this.aimGroundCoralBtn = t;
        this.aimGroundCoralBtn.onTrue(new InstantCommand(() -> {
            System.out.println("Aim Ground Coral");
        }));
    }

    public void setIntakeTrigger(Trigger t) {
        // 开启intake，shoot，Aglea模式下是挑球
        if (t == null) {
            return;
        }
        if (intakeTrigger != null) {
            DriverStation.reportError("setIntakeTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }
        intakeTrigger = t;
        intakeTrigger.onTrue(new InstantCommand(() -> {
            if (getIsInAgleaMode()) {
                m_elevator.setOffset(-1.5);
                m_intake.toggleBallIntake(true);
            }
            else{
                m_intake.toggleCoralIntake();
            }
        }))
        .onFalse(new InstantCommand(() -> {
            if (getIsInAgleaMode()) {
                m_elevator.clearOffset();
                m_intake.toggleBallIntake(false);
            }
        }))
        ;
    }

    public void setGroundIntakeSwitchTrigger(Trigger t) {
        if (t == null) {
            return;
        }
        if (groundIntakeSwitchTrigger != null) {
            DriverStation.reportError("setGroundIntakeSwitchTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }
        groundIntakeSwitchTrigger = t;
        groundIntakeSwitchTrigger.onTrue(new InstantCommand(() -> {
            // 只有在READY_FOR_LOAD_GROUND_CORAL状态下，才允许切换地吸开关
            if (curState == STATE.READY_FOR_LOAD_GROUND_CORAL) {
                m_groundIntake.toggle();
            }
        }));
    }

    public void setLnTrigger(Trigger l1, Trigger l2, Trigger l3, Trigger l4) {
        if (l1 != null) {
            if (L1Trigger != null) {
                DriverStation.reportError("setLnTrigger L1 trigger bind multiple times. Please check your code!", true);
            }
            else {
                L1Trigger = l1;
                L1Trigger.onTrue(new InstantCommand(() -> {
                    ControlPadHelper.setControlPadInfoData(-10, 0, -10);
                    if (getIsCarryingCoral()) {
                        setState(STATE.L1);
                    }                    
                }));
            }
        }

        if (l2 != null) {
            if (L2Trigger != null) {
                DriverStation.reportError("setLnTrigger L2 trigger bind multiple times. Please check your code!", true);
            }
            else {
                L2Trigger = l2;
                L2Trigger.onTrue(new InstantCommand(() -> {
                    ControlPadHelper.setControlPadInfoData(-10, 1, -10);
                    // if (getIsCarryingCoral()) {
                    //     setState(STATE.L2);
                    // }
                }));
            }
        }

        if (l3 != null) {
            if (L3Trigger != null) {
                DriverStation.reportError("setLnTrigger L3 trigger bind multiple times. Please check your code!", true);
            }
            else {
                L3Trigger = l3;
                L3Trigger.onTrue(new InstantCommand(() -> {
                    ControlPadHelper.setControlPadInfoData(-10, 2, -10);
                    // if (getIsCarryingCoral()) {
                    //     setState(STATE.L3);
                    // }
                }));
            }
        }

        if (l4 != null) {
            if (L4Trigger != null) {
                DriverStation.reportError("setLnTrigger L4 trigger bind multiple times. Please check your code!", true);
            }
            else {
                L4Trigger = l4;
                L4Trigger.onTrue(new InstantCommand(() -> {
                    ControlPadHelper.setControlPadInfoData(-10, 3, -10);
                    // if (getIsCarryingCoral()) {
                    //     setState(STATE.L4);
                    // }
                }));
            }
        }
    }

    public void expendGroundIntake() {
        m_groundIntake.expend();;
    }

    public void makeSureGroundIntakeRetracted() {
        m_groundIntake.makeSureRetracted();
    }

    public void setIntakeSource(boolean isFromGround) {
        // 设置Intake来源，地吸还是漏斗
        
        if (!m_groundIntake.isIdle()) {
            // 如果地吸在非待机状态，则不允许切换
            return;
        }
        m_isIntakeFromGround = isFromGround;
        if (m_isIntakeFromGround && curState == STATE.READY_FOR_LOAD_UP_CORAL) {
            setState(STATE.READY_FOR_LOAD_GROUND_CORAL);
        }
        if (!m_isIntakeFromGround && curState == STATE.READY_FOR_LOAD_GROUND_CORAL) {
            setState(STATE.READY_FOR_LOAD_UP_CORAL);
        }
    }

    public void setSwitchIntakeSourceTrigger(Trigger t) {
        if (t == null) {
            return;
        }
        if (switchIntakeSourceTrigger != null) {
            DriverStation.reportError("switchIntakeSourceTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }
        switchIntakeSourceTrigger = t;
        switchIntakeSourceTrigger.onTrue(new InstantCommand(() -> {
            setIntakeSource(!m_isIntakeFromGround);
        }));
    }

    public void setElevatorLevelUpTrigger(Trigger t) {
        if (t == null) {
            return;
        }
        if (elevatorLevelUpTrigger != null) {
            DriverStation.reportError("elevatorLevelUpTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }
        elevatorLevelUpTrigger = t;
        elevatorLevelUpTrigger.onTrue(new InstantCommand(() -> {
            makeElevatorLevelUpOrDown(true);
        }));
    }

    public void setElevatorLevelDownTrigger(Trigger t) {
        if (t == null) {
            return;
        }
        if (elevatorLevelDownTrigger != null) {
            DriverStation.reportError("elevatorLevelDownTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }
        elevatorLevelDownTrigger = t;
        elevatorLevelDownTrigger.onTrue(new InstantCommand(() -> {
            makeElevatorLevelUpOrDown(false);
        }));
    }

    public void setSwitchCoralAndBallTrigger(Trigger t) {
        if (t == null) {
            return;
        }
        if (switchCoralAndBallTrigger != null) {
            DriverStation.reportError("setSwitchCoralAndBallTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }
        switchCoralAndBallTrigger = t;
        switchCoralAndBallTrigger.onTrue(new InstantCommand(() -> {
            // 这里切换Coral或者Aglea模式，只简单判断当前是否在Ball1或者Ball2模式下，
            // 进一步的切换条件判断在setState函数里会处理，比如持有Coral的时候不允许切换等
            if (getIsInAgleaMode()) {
                setState(m_isIntakeFromGround ? STATE.READY_FOR_LOAD_GROUND_CORAL : STATE.READY_FOR_LOAD_UP_CORAL);
            }
            else {
                setState(STATE.BALL1);
            }
            
        }));
    }

    public void setGroundIntakeOutTakeTrigger(Trigger t) {
        if (t == null) {
            return;
        }
        if (groundIntakeOutTakeTrigger != null) {
            DriverStation.reportError("setGroundIntakeOutTakeTrigger trigger bind multiple times. Please check your code!", true);
            return;
        }
        groundIntakeOutTakeTrigger = t;
        groundIntakeOutTakeTrigger.onTrue(new InstantCommand(() -> {
            // TODO: 地吸反转吐Coral
        }));
    }

    public void setElevatorTuningUpTrigger(Trigger t) {
        if (t == null || elevatorTuningUpTrigger != null) {
            return;
        }
        elevatorTuningUpTrigger = t;
        t.onTrue(new InstantCommand(() -> {
            m_elevator.toggleTuningUp();
        }));
    }


    public void setElevatorTuningDownTrigger(Trigger t) {
        if (t == null || elevatorTuningDownTrigger != null) {
            return;
        }
        elevatorTuningDownTrigger = t;
        t.onTrue(new InstantCommand(() -> {
            m_elevator.toggleTuningDown();
        }));
    }

    public void setArmTuningUpTrigger(Trigger t) {
        if (t == null || armTuningUpTrigger != null) {
            return;
        }
        armTuningUpTrigger = t;
        t.onTrue(new InstantCommand(() -> {
            System.out.println("Arm up --------------------------------");
            m_turningArm.toggleTuningUp();
        }));
    }

    public void setArmTuningDownTrigger(Trigger t) {
        if (t == null || armTuningDownTrigger != null) {
            return;
        }
        armTuningDownTrigger = t;
        t.onTrue(new InstantCommand(() -> {
            m_turningArm.toggleTuningDown();
        }));
    }

    public void setResetCanCodePositionTrigger(Trigger t) {
        if (this.resetCanCodePositionBtn != null) {
            return;
        }
        if (t != null) {
            this.resetCanCodePositionBtn = t;
            this.resetCanCodePositionBtn.onTrue(new InstantCommand(() -> {
                if (Robot.inst.isTestEnabled()) {
                    System.out.println("reset elevator, ground intake and turning arm's cancoder position to 0");
                    m_elevator.resetCancodePosition();
                    m_turningArm.resetCancodePosition();
                    m_groundIntake.resetCancodePosition();
                }
                else {
                    System.out.println("No No No, only working at TEST mode! reset elevator and turning arm's cancoder position to 0");
                }

            }));
        }
    }

    private Trigger lockElevatorTrigger;
    private Trigger unlockElevatorTrigger;
    public void setLockElevatorTrigger(Trigger t) {
        if (t == null || lockElevatorTrigger != null) {
            return;
        }
        lockElevatorTrigger = t;
        lockElevatorTrigger.onTrue(new InstantCommand(() -> {
            m_elevator.lockMotor();
            m_turningArm.lockMotor();
            m_groundIntake.lockMotor();
        }));
    }

    public void setUnlockElevatorTrigger(Trigger t) {
        if (t == null || unlockElevatorTrigger != null) {
            return;
        }
        unlockElevatorTrigger = t;
        unlockElevatorTrigger.onTrue(new InstantCommand(() -> {
            m_elevator.unlockMotor();
            m_turningArm.unlockMotor();
            m_groundIntake.unlockMotor();
        }));
    }

    void initDebug() {
        if (isDebugEnabled) {
            // ControlPadHelper.DebugCtrl.onZero.onTrue(new InstantCommand(() -> {
            //     setState(STATE.ZERO);
            // }));
            ControlPadHelper.DebugCtrl.onReadyForloadCoral.onTrue(new InstantCommand(() -> {
                setState(m_isIntakeFromGround ? STATE.READY_FOR_LOAD_GROUND_CORAL : STATE.READY_FOR_LOAD_UP_CORAL);
            }));
            ControlPadHelper.DebugCtrl.onL1.onTrue(new InstantCommand(() -> {
                setState(STATE.L1);
            }));
            ControlPadHelper.DebugCtrl.onL2.onTrue(new InstantCommand(() -> {
                setState(STATE.L2);
            }));
            ControlPadHelper.DebugCtrl.onL3.onTrue(new InstantCommand(() -> {
                // setState(STATE.L3);
            }));
            ControlPadHelper.DebugCtrl.onL4.onTrue(new InstantCommand(() -> {
                setState(STATE.L4);
            }));
            ControlPadHelper.DebugCtrl.onBall1.onTrue(new InstantCommand(() -> {
                setState(STATE.BALL1);
            }));
            ControlPadHelper.DebugCtrl.onBall2.onTrue(new InstantCommand(() -> {
                setState(STATE.BALL2);
            }));

            ControlPadHelper.DebugCtrl.shoot.onTrue(new InstantCommand(() -> {
                startShoot();
            }));
            // ControlPadHelper.DebugCtrl.onLoadCoral.onTrue(new InstantCommand(() -> {
            //     isCarryingCoralFromDebug = true;
            // })).whileFalse(new InstantCommand(() -> {
            //     isCarryingCoralFromDebug = false;
            // }));
            // ControlPadHelper.DebugCtrl.onLoadBall.onTrue(new InstantCommand(() -> {
            //     isCarryingBallFromDebug = true;
            // })).whileFalse(new InstantCommand(() -> {
            //     isCarryingBallFromDebug = false;
            // }));
        }
    }

    @Override
    public void initialize() {

        if (Robot.inst.isTeleop() || Robot.inst.isAutonomous())
        {
            System.out.println("UpperSystemCmd init in telop or auto");
            m_elevator.init();
            m_turningArm.init(); 
            m_intake.init();
            m_groundIntake.init();
            m_candle.init();

            // 下面两行要联动，m_isIntakeFromGround改成false的时候，setState要变成READY_FOR_LOAD_UP_CORAL
            m_isIntakeFromGround = true;
            setState(STATE.READY_FOR_LOAD_GROUND_CORAL);
        }
        else if (Robot.inst.isTest()){
            System.out.println("UpperSystemCmd init in test");
            m_elevator.initInTestMode();
            m_turningArm.initInTestMode();
        }
    }

    @Override
    public void execute() {
        runState();
        updateState();
        updateLeds();
        telemetry();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("UpperSystemCmd END ======>" + interrupted);
        m_elevator.onDisable();
        m_turningArm.onDisable();
        m_groundIntake.onDisable();
        m_candle.onDisable();
    }
    

    @Override
    public boolean isFinished() {
        return false;
    }

    // public void onDisable() {
    // m_elevator.onDisable();
    // m_turningArm.onDisable();
    // }

    public boolean getIsGroundIntakeCoralIn() {
        return m_groundIntake.isCoralIn();
    }

    private boolean getIsCarryingBall() {
        // TODO: get this from sesnor
        // if (isDebugEnabled) {
        //     return isCarryingBallFromDebug;
        // }
        return false;
    }

    private boolean getIsCarryingCoral() {
        if (m_intake.getIsCarryingCoral()) {
            return true;
        }

        // if (isDebugEnabled) {
        //     return isCarryingCoralFromDebug;
        // }

        return false;
    }

    private boolean getIsInAgleaMode() {
        return curState == STATE.BALL1 || curState == STATE.BALL2;
    }

    private void setState(STATE newState) {
        System.out.println("UpperSystem2025Cmd::setState: try set: " + curState + " -> " + newState
                + " isCarryingCoral: " + getIsCarryingCoral() + " isCarryingBall: " + getIsCarryingBall());
        if (curState == newState) {
            return;
        }

        if (curState == STATE.ZERO && (newState != STATE.READY_FOR_LOAD_GROUND_CORAL && newState != STATE.READY_FOR_LOAD_UP_CORAL)) {
            // if we are in zero state, we can only go to READY_FOR_LOAD_CORAL state or READY_FOR_LOAD_UP_CORAL
            return;
        }

        if (newState == STATE.L1 || newState == STATE.L2 || newState == STATE.L3 || newState == STATE.L4) {
            if (curState != STATE.READY_FOR_LOAD_GROUND_CORAL
                    && curState != STATE.READY_FOR_LOAD_UP_CORAL
                    && curState != STATE.L1
                    && curState != STATE.L2
                    && curState != STATE.L3
                    && curState != STATE.L4) {
                // if we are not in READY_FOR_LOAD_CORAL, READY_FOR_LOAD_UP_CORAL, L1, L2, L3, L4, 
                // refuse to go to L1, L2, L3, L4
                return;
            }
            if (!getIsCarryingCoral() || getIsCarryingBall()) {
                // if we are not carrying Coral or are carrying ball, refuse to go to L1, L2,
                // L3, L4
                return;
            }
        }

        if (newState == STATE.BALL1 || newState == STATE.BALL2) {
            if (curState != STATE.READY_FOR_LOAD_GROUND_CORAL && curState != STATE.READY_FOR_LOAD_UP_CORAL && curState != STATE.BALL1 && curState != STATE.BALL2) {
                // if we are not in READY_FOR_LOAD_BALL, refuse to go to BALL1
                return;
            }
            if (getIsCarryingCoral()) {
                // if we are not carrying ball or are carrying coral, refuse to go to BALL1
                return;
            }
        }

        if (newState == STATE.READY_FOR_LOAD_GROUND_CORAL || newState == STATE.READY_FOR_LOAD_UP_CORAL) {
            if (getIsCarryingBall() || getIsCarryingCoral()) {
                // if we are carrying ball or coral, refuse to go to READY_FOR_LOAD_CORAL or READY_FOR_LOAD_UP_CORAL
                return;
            }
        }

        // if (newState == STATE.BALL1 && curState == STATE.READY_FOR_LOAD_CORAL) {
        //     if (getIsCarryingBall() || getIsCarryingCoral()) {
        //         // if we are carrying ball or coral, refuse to go to READY_FOR_LOAD_BALL
        //         return;
        //     }
        // }

        curRunningState = RUNNING_STATE.NEW_SET;
        lastState = curState;
        curState = newState;

        System.out.println("UpperSystem2025Cmd::setState: try set: comfirmed");
    }


    private int curRunningDir = 0;


    // private void doStateAction__old2(TA_STATE taState, EV_STATE evState) {
    //     boolean isNeedDodge = false;
    //     boolean isBall2Down = false;
    //     if (curRunningState == RUNNING_STATE.NEW_SET)
    //     {
    //         STATE sx = lastState;
    //         STATE sy = curState;
    //         int tx = sx.ordinal();
    //         int ty = sy.ordinal(); //

    //         curRunningDir = table[ty][tx];
    //         System.out.println("doStateAction NEW_SET: lastState: " + lastState.name() + ", curState: " + curState.name());
    //         System.out.println("doStateAction NEW_SET: tx: " + tx + " ty: " + ty);
    //         System.out.println("doStateAction NEW_SET: curRunningDir is " + curRunningDir);

    //         if ((lastState != STATE.BALL1 && lastState != STATE.BALL2) &&
    //             (curState == STATE.BALL1 || curState == STATE.BALL2)) {
    //             isNeedDodge = false;
    //         }
    //         else if ((lastState == STATE.BALL1 || lastState == STATE.BALL2) &&
    //             (curState == STATE.BALL1 || curState == STATE.BALL2)) {
    //             isNeedDodge = false;
    //         }
    //         else if ((lastState == STATE.BALL1 || lastState == STATE.BALL2) &&
    //             (curState == STATE.READY_FOR_LOAD_GROUND_CORAL)) {
    //             isNeedDodge = false;
    //             isBall2Down = true;
    //         }
    //         else {
    //             double[] dodgePosList = m_elevator.getDodgePosOrderFromUp2Down();
    //             double elevatorStartPos = m_elevator.getCurPos();
    //             double elevatorEndPos = m_elevator.getStatePos(evState);
    //             for (int i = 0; i < dodgePosList.length; i++) {
    //                 if (MiscUtils.isBeween(dodgePosList[i], elevatorStartPos, elevatorEndPos)) {
    //                     isNeedDodge = true;
    //                     break;
    //                 }
    //             }
    //         }


    //         System.out.println("doStateAction NEW_SET: isNeedDodge: " + isNeedDodge);
    //         if (m_actionRunner != null) {
    //             m_actionRunner.cancel();
    //         }
    //         m_actionRunner = new ActionRunner();

    //         if (curRunningDir == -1) {
    //             // down
    //             if (isNeedDodge) {
    //                 m_actionRunner.addQueueAction(
    //                     () -> { // init
    //                     m_turningArm.setState(TA_STATE.DODGE);
    //                     },
    //                     () -> { // update
    //                     },
    //                     () -> {},   // onCancel
    //                     () -> { // endCondition
    //                         return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
    //                     }
    //                 );
    //             }
    //             m_actionRunner.addQueueAction(
    //                 () -> { // init
    //                     m_elevator.setState(evState);
    //                 },
    //                 () -> { // update
    //                 },
    //                 () -> {},   // onCancel
    //                 () -> { // endCondition
    //                     return m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE;
    //                 }
    //             )
    //             .addConditionAction(
    //                 () -> { // init
    //                     m_turningArm.setState(taState);
    //                 },
    //                 () -> { // update
    //                 },
    //                 () -> {},   // onCancel
    //                 // startCondition
    //                 isNeedDodge ?
    //                 () -> { 
    //                     return m_elevator.isCurPosBelow(Constants.Elevator.downDodgePos);
    //                 } : (isBall2Down ? () -> {
    //                     return m_elevator.isCurPosBelow(m_elevator.getStatePos(EV_STATE.BALL1));
    //                 } :
    //                 () -> true),
    //                 () -> { // endCondition
    //                     return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
    //                 }
    //             )
    //             // .addQueueAction(                    
    //             //     () -> { // init
    //             //         m_turningArm.setState(taState);
    //             //     },
    //             //     () -> { // update
    //             //     },
    //             //     () -> {},   // onCancel
    //             //     () -> { // endCondition
    //             //         return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
    //             // })
    //             .start();
    //         }
    //         else if (curRunningDir == 1) {
    //             // up
    //             if (isNeedDodge) {
    //                 m_actionRunner.addQueueAction(
    //                     () -> { // init
    //                     m_turningArm.setState(TA_STATE.DODGE);
    //                     },
    //                     () -> { // update
    //                     },
    //                     () -> {},   // onCancel
    //                     () -> { // endCondition
    //                         return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
    //                     }
    //                 );
    //             }
    //             m_actionRunner.addQueueAction(
    //                 () -> { // init
    //                     m_elevator.setState(evState);
    //                 },
    //                 () -> { // update
    //                 },
    //                 () -> {},   // onCancel
    //                 () -> { // endCondition
    //                     return m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE;
    //                 }
    //             )
    //             .addConditionAction(
    //                 () -> { // init
    //                     m_turningArm.setState(taState);
    //                 },
    //                 () -> { // update
    //                 },
    //                 () -> {},   // onCancel
    //                 // startCondition
    //                 isNeedDodge ? 
    //                 () -> { 
    //                     return m_elevator.isCurPosUpper(Constants.Elevator.upDodgePos);
    //                 } : 
    //                 () -> true,
    //                 () -> { // endCondition
    //                     return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
    //                 }
    //             )
    //             // .addQueueAction(                    
    //             //     () -> { // init
    //             //         m_turningArm.setState(taState);
    //             //     },
    //             //     () -> { // update
    //             //     },
    //             //     () -> {},   // onCancel
    //             //     () -> { // endCondition
    //             //         return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
    //             //     }
    //             // )
    //             .start();
    //         }

    //         curRunningState = RUNNING_STATE.RUNNING;
    //     }


    //     if (curRunningState == RUNNING_STATE.RUNNING) {
    //         m_actionRunner.update();
    //         if (m_actionRunner.getIsDone()) {
    //             curRunningState = RUNNING_STATE.DONE;
    //         }
    //     }
    // }

    private void doStateAction(TA_STATE taState, EV_STATE evState) {

        if (curRunningState == RUNNING_STATE.NEW_SET)
        {
            curRunningDir = getMovingDir(lastState, curState);
            if (m_actionRunner != null) {
                m_actionRunner.cancel();
            }
            m_actionRunner = new ActionRunner();

            if (curRunningDir == -1) {
                m_actionRunner.addQueueAction(
                        () -> { // init
                            m_elevator.setState(evState);
                        },
                        () -> { // update
                        },
                        () -> {},   // onCancel
                        () -> { // endCondition
                            return m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE;
                        }
                    )
                    .addQueueAction(
                        () -> { // init
                            m_turningArm.setState(taState);
                        },
                        () -> { // update
                        },
                        () -> {},   // onCancel
                        () -> { // endCondition
                            return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
                        }
                    )
                    .start();
            }
            else if (curRunningDir == 1 || curRunningDir == 0) {
                m_actionRunner
                .addQueueAction(
                    () -> { // init
                        m_turningArm.setState(taState);
                    },
                    () -> { // update
                    },
                    () -> {},   // onCancel
                    () -> { // endCondition
                        return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
                    }
                )
                .addQueueAction(
                    () -> { // init
                        m_elevator.setState(evState);
                    },
                    () -> { // update
                    },
                    () -> {},   // onCancel
                    () -> { // endCondition
                        return m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE;
                    }
                )
                .start();
            }
            curRunningState = RUNNING_STATE.RUNNING;
        }

        if (curRunningState == RUNNING_STATE.RUNNING) {
            m_actionRunner.update();
            if (m_actionRunner.getIsDone()) {
                curRunningState = RUNNING_STATE.DONE;
            }
        }
    }

    private void runState() {
        switch (curState) {
            case ZERO:
                doStateAction(TA_STATE.ZERO, EV_STATE.ZERO);
                // updateStateZero();
                break;
            case READY_FOR_LOAD_GROUND_CORAL:
                doStateAction(TA_STATE.GI, EV_STATE.BASE);
                // updateStateReadyForLoadCoral();
                break;
            case READY_FOR_LOAD_UP_CORAL:
                doStateAction(TA_STATE.UI, EV_STATE.BASE);
                break;
            case L1:
                doStateAction(TA_STATE.L1, EV_STATE.L1);
                // updateStateL1();
                break;
            case L2:
                doStateAction(TA_STATE.L2, EV_STATE.L2);
                // updateStateL2();
                break;
            case L3:
                doStateAction(TA_STATE.L3, EV_STATE.L3);
                // updateStateL3();
                break;
            case L4:
                doStateAction(TA_STATE.L4, EV_STATE.L4);
                // updateStateL4();
                break;
            case BALL1:
                doStateAction(TA_STATE.BALL1, EV_STATE.BALL1);
                break;
            case BALL2:
                doStateAction(TA_STATE.BALL2, EV_STATE.BALL2);
                break;
            default:
                break;
        }
    }

    private void updateState() {
        // check if state should be changed
        // some state change bind to trigger. these triggers are set in the constructor
        if (lastState == STATE.ZERO && curRunningState != RUNNING_STATE.DONE) {
            // we are going from zero to other state, we should not change state. It's
            // dangerous!!!!!!!
            return;
        }
        switch (curState) {
            case ZERO:
                break;
            case READY_FOR_LOAD_GROUND_CORAL:
            case READY_FOR_LOAD_UP_CORAL:
                if (getIsCarryingCoral()) {
                    setState(STATE.L1);
                }
                break;
            case L1:
            case L2:
            case L3:
            case L4:
                if (!getIsCarryingCoral()) {
                    setState(m_isIntakeFromGround ? STATE.READY_FOR_LOAD_GROUND_CORAL : STATE.READY_FOR_LOAD_UP_CORAL);
                }

                if (m_turningArm.getIsInCoralSafeZone()) {
                    m_intake.setCanAdjusetCoralPosition(true);
                }
                else {
                    m_intake.setCanAdjusetCoralPosition(false);
                }
                break;
            default:
                break;
        }
    }

    private void updateLeds() {
        if (getIsCarryingCoral()) {
            this.m_candle.showCarryingCoral();
        }
        else {
            this.m_candle.showIdle();
        }

        ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
        if (info == null) {
            this.m_candle.clearLn();
        }
        else {
            if (info.level == -1) {
                // show aglea
            }
            else if (info.level == 0) {
                this.m_candle.showL1();
            }
            else {
                if (info.branch != 0) {
                    boolean isLeft = (info.branch == -1);
                    switch ((int)info.level) {
                        case 1:
                            m_candle.showL2(isLeft);
                            break;
                        case 2:
                            m_candle.showL3(isLeft);
                            break;
                        case 3:
                            m_candle.showL4(isLeft);
                            break;
                    }                    
                }
            }
            
        }

    }

    public void setStateLn() {
        ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
        if (info == null) {
            return;
        }
        STATE newState = STATE.NONE;
        switch ((int)info.level) {
            case 0:
                newState = STATE.L1;
                break;
            case 1:
                newState = STATE.L2;
                break;
            case 2:
                newState = STATE.L3;
                break;
            case 3:
                newState = STATE.L4;
                break;
        }
        if (newState != STATE.NONE) {
            setState(newState);
        }
    }

    public void setStateL1() {
        setState(STATE.L1);
    }

    public void setStateL2() {
        setState(STATE.L2);
    }

    public void setStateL3() {
        setState(STATE.L3);
    }

    public void setStateL4() {
        setState(STATE.L4);
    }

    public void startIntake() {
        m_intake.startIntake();
    }

    public void startShoot() {
        m_intake.startShoot();
    }

    public void setStateBall1() {
        setState(STATE.BALL1);
    }

    public void setStateBall2() {
        setState(STATE.BALL2);
    }

    public long getNearestSeenCoralAprilTag1() {
        LimelightHelpers.LimelightResults left = LimelightHelpers.getLatestResults(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT);
        LimelightHelpers.LimelightResults right = LimelightHelpers.getLatestResults(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT);

        Map<Long, LimelightHelpers.LimelightTarget_Fiducial> map = new HashMap<>();
        for (int i = 0; i < left.targets_Fiducials.length; ++i) {
            LimelightHelpers.LimelightTarget_Fiducial data = left.targets_Fiducials[i];
            long id = Math.round(data.fiducialID);
            if (!map.containsKey(id)) {
                map.put(id, data);
            }
        }
        for (int i = 0; i < right.targets_Fiducials.length; ++i) {
            LimelightHelpers.LimelightTarget_Fiducial data = right.targets_Fiducials[i];
            long id = Math.round(data.fiducialID);
            if (!map.containsKey(id)) {
                map.put(id, data);
            }
        }

        if (map.isEmpty()) {
            System.out.println("No seen coral AprilTag");
            return -1;
        }
        LimelightHelpers.LimelightTarget_Fiducial[] list = (LimelightHelpers.LimelightTarget_Fiducial[])map.values().toArray();

        Pose2d robotPos= m_chassis.getPose();
        long minDistanceApId = -1;
        double minDistance = Double.MAX_VALUE;
        for (LimelightHelpers.LimelightTarget_Fiducial data : list) {
            long apId = Math.round(data.fiducialID);
            Constants.FieldInfo.APInfo ap = Constants.FieldInfo.AP_MAP.get(apId);
            Pose2d apPose = new Pose2d(ap.getX(), ap.getY(), new Rotation2d(Units.degreesToRadians(ap.getTheta())));

            double distance = robotPos.getTranslation().getDistance(apPose.getTranslation());
            if (distance < minDistance) {
                minDistanceApId = (long)ap.getId();
            }
        }

        return minDistanceApId;
    }



    public long getNearestSeenCoralAprilTag() {
        Constants.FieldInfo.APInfo[] apList = MiscUtils.getApInfoByAlliance();
        if (apList.length == 0) {
            return -1;
        }
        Pose2d robotPos= m_chassis.getPose();
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

    private Command getMoveToCoralCmd() {
        long targetApId = -1;
        if (DriverStation.isAutonomous()) {
            ControlPadHelper.ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
            targetApId = info.aprilTagId;
        }
        else {
            targetApId = getNearestSeenCoralAprilTag();
            if (targetApId == -1) {
                return new InstantCommand(() -> {
                    System.out.println("getMoveToCoralCmd: no aprilTag found!!!!");
                });
            }
    
            ControlPadHelper.setControlPadInfoData(targetApId, -10l, -10l);
        }


        ControlPadHelper.ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
        if (info == null) {
            return new InstantCommand(() -> {
                System.out.println("getMoveToCoralCmd: getControlPadInfo error!!!!!!!!!!!!!!!!");
            });
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

        if (autoMoveCmd != null) {
            autoMoveCmd.cancel();
            autoMoveCmd = null;
        }

        // for test
        SmartDashboard.putString("MoveToCoralState", String.format("AP: %d, level: %d, branch: %d, targetPos: %s", apId, info.level, info.branch, targetPos.toString()));
        // return null;
        autoMoveCmd = m_chassis.autoMoveToPoseCommand(targetPos);
        return autoMoveCmd;
    }

    private void moveToGroundCoral() {
        if (autoMoveCmd != null) {
            autoMoveCmd.cancel();
            autoMoveCmd = null;
        }
        autoMoveCmd = new GoToCoralCmd(m_chassis);
    }

    // return: -1 down, 1 up, 0 evelvator no move
    private int getMovingDir(STATE pre, STATE cur) {
        switch (pre) {
            case NONE:
            case ZERO:
            case READY_FOR_LOAD_GROUND_CORAL:
            case READY_FOR_LOAD_UP_CORAL:
            {
                switch (cur) {
                    case NONE:
                    case ZERO: 
                    case READY_FOR_LOAD_GROUND_CORAL: 
                    case READY_FOR_LOAD_UP_CORAL:
                        return 0;
                    case L1: 
                    case L2: 
                    case L3: 
                    case L4: 
                        return 1;
                    case BALL1: 
                    case BALL2:
                        return 1;
                }
                break;
            }
            case L1: {
                switch (cur) {
                    case NONE:
                    case ZERO: 
                    case READY_FOR_LOAD_GROUND_CORAL: 
                    case READY_FOR_LOAD_UP_CORAL:
                        return -1;
                    case L1: 
                        return 0;
                    case L2: 
                    case L3: 
                    case L4: 
                        return 1;
                    case BALL1: 
                    case BALL2:
                        return 0;
                }
                break;
            }
            case L2: {
                switch (cur) {
                    case NONE:
                    case ZERO: 
                    case READY_FOR_LOAD_GROUND_CORAL: 
                    case READY_FOR_LOAD_UP_CORAL:
                    case L1: 
                        return -1;
                    case L2: 
                        return 0;
                    case L3: 
                    case L4: 
                        return 1;
                    case BALL1: 
                    case BALL2:
                        return 0;
                }
                break;
            }
            case L3: {
                switch (cur) {
                    case NONE:
                    case ZERO: 
                    case READY_FOR_LOAD_GROUND_CORAL: 
                    case READY_FOR_LOAD_UP_CORAL:
                    case L1: 
                    case L2: 
                        return -1;
                    case L3: 
                        return 0;
                    case L4: 
                        return 1;
                    case BALL1: 
                    case BALL2:
                        return 0;
                }
                break;
            }
            case L4: {
                switch (cur) {
                    case NONE:
                    case ZERO: 
                    case READY_FOR_LOAD_GROUND_CORAL: 
                    case READY_FOR_LOAD_UP_CORAL:
                    case L1: 
                    case L2: 
                    case L3: 
                        return -1;
                    case L4: 
                        return 0;
                    case BALL1: 
                    case BALL2:
                        return 0;
                }
                break;
            }
            case BALL1: {
                switch (cur) {
                    case NONE:
                    case ZERO: 
                    case READY_FOR_LOAD_GROUND_CORAL: 
                    case READY_FOR_LOAD_UP_CORAL:
                        return -1;
                    case L1: 
                    case L2: 
                    case L3: 
                    case L4: 
                        return 0;
                    case BALL1: 
                        return 0;
                    case BALL2:
                        return 1;
                }
                break;
            }
            case BALL2: {
                switch (cur) {
                    case NONE:
                    case ZERO: 
                    case READY_FOR_LOAD_GROUND_CORAL: 
                    case READY_FOR_LOAD_UP_CORAL:
                        return -1;
                    case L1: 
                    case L2: 
                    case L3: 
                    case L4: 
                        return 0;
                    case BALL1: 
                        return -1;
                    case BALL2:
                        return 0;
                }
                break;
            }
            default:
                return 0;
        }
        return 0;
    }

    private void makeElevatorLevelUpOrDown(boolean isUp) {
        if (!getIsInAgleaMode() && !getIsCarryingCoral()) {
            return;
        }

        STATE newState = getNextElevatorLevel(isUp);
        if (newState != STATE.NONE) {
            setState(newState);
        }
    }

    private STATE getNextElevatorLevel(boolean isUp) {
        switch (curState) {
            case L1:
                return isUp ? STATE.L2 : STATE.L1;
            case L2:
                return isUp ? STATE.L3 : STATE.L1;
            case L3:
                return isUp ? STATE.L4 : STATE.L2;
            case L4:
                return isUp ? STATE.L4 : STATE.L3;
            case BALL1:
                return isUp ? STATE.BALL2 : STATE.BALL1;
            case BALL2:
                return isUp ? STATE.BALL2 : STATE.BALL1;
            default:
                break;
        }

        return STATE.NONE;
    }

    protected void telemetry() {

        // DON'T DELETE BELOW SmartDashboard push, cause ControlPad is using them!!!
        // DON'T DELETE BELOW SmartDashboard push, cause ControlPad is using them!!!
        // DON'T DELETE BELOW SmartDashboard push, cause ControlPad is using them!!!
        SmartDashboard.putString("US2025Cmd_State", "state: " + curState + " running state: " + curRunningState);
        // SmartDashboard.putString("US2025Cmd_CarryingState",
        //     "isCarryingCoral: " + m_intake.getIsCarryingCarol() + " isCarryingBall: " + getIsCarryingBall());
        SmartDashboard.putString("US2025Cmd_CarryingState",
            "isCarryingCoral: " + getIsCarryingCoral() + " isCarryingBall: " + getIsCarryingBall());

        SmartDashboard.putString("US2025Cmd_Sub_Running_State", "arm: " + m_turningArm.getCurRunningState() + " elevator: " + m_elevator.getCurRunningState() + " elevator state: " + m_elevator.getState());
        // DON'T DELETE UP CODES
        // DON'T DELETE UP CODES
        // DON'T DELETE UP CODES
    }

    // for debug
    public void setGISensorTrigger(Trigger t) {
        t.onTrue(new InstantCommand(() -> {
            m_groundIntake.toggleDebugSensor();
        }));
    }
}
