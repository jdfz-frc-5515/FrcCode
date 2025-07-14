// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.Cursor;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.ProtectionDomain;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GIntakeConstants;
import frc.robot.subsystems.TurningArm2025.TurningArm2025.TA_STATE;
import frc.robot.utils.MiscUtils;

public class GroundIntakeSubsystem extends SubsystemBase {
    // public enum GI_STATE___OLD {
    //     NONE,
    //     RETRACT,
    //     EXPAND,
    //     OUT_TAKE,   // 吐给上层Intake
    // }

    // public enum GI_RUNNING_STATE {
    //     READY,
    //     RUNNING,
    //     DONE,
    // }

    public enum GI_STATE {
        IDLE,
        EXPANDING,
        WAIT_FOR_CORAL,
        RETRACTING_WITHOUT_CORAL,   // -> IDLE
        RETRACTING_WITH_CORAL,      // -> WAITING_FOR_REVERSE_INTAKE
        WAITING_FOR_REVERSE_INTAKE,
        REVERSE_INTAKING,           // -> IDLE   
    }

    private DigitalInput coralSensor = new DigitalInput(2);
    private final TalonFX m_turnMotor = new TalonFX(GIntakeConstants.GIntakeTurnID, GIntakeConstants.canBusName);
    private final TalonFX m_driveMotor = new TalonFX(GIntakeConstants.GIntakeDriveID, GIntakeConstants.canBusName);
    private final CANcoder m_CANcoder = new CANcoder(GIntakeConstants.GIntakeCCID, GIntakeConstants.canBusName);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    boolean isDebugSensorOn = false;
    boolean debugSensored = false;

    private Runnable startUpperIntake;
    private Runnable stopUppeerIntake;

    private final double NONE_POS = -9999;
    private final double threshold = 0.05;

    double intakeMotorSpeed = 0;

    GI_STATE curState = GI_STATE.IDLE;
    // GI_RUNNING_STATE curRunningState = GI_RUNNING_STATE.READY;

    
    // final int DELAY_FRAME = 20;
    // int delayTime = DELAY_FRAME;

    // final int REVERSE_INTAKE_MAX_FRAME = 100;
    // int delayRevereIntakeFrame = -1;

    final int MAX_WAIT_FOR_REVERSE_FRAME = 20;
    int waitForReverseFrame = -1;

    final int MAX_REVERSER_INTAKE_FRAME = 50;
    int reverserIntakeCountdoownFrame = -1;

    private TalonFXConfiguration getTurnMotorConfiguration(boolean lockMotor) {
        TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
        turnMotorConfig.Slot0.kP = GIntakeConstants.turnMotorConst.kP;
        turnMotorConfig.Slot0.kI = GIntakeConstants.turnMotorConst.kI;
        turnMotorConfig.Slot0.kD = GIntakeConstants.turnMotorConst.kD;
        turnMotorConfig.Slot0.kS = GIntakeConstants.turnMotorConst.kS;
        turnMotorConfig.Slot0.kV = GIntakeConstants.turnMotorConst.kV;
        turnMotorConfig.Slot0.kA = GIntakeConstants.turnMotorConst.kA;
        turnMotorConfig.Feedback.FeedbackRemoteSensorID = GIntakeConstants.GIntakeCCID;
        turnMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnMotorConfig.MotorOutput.NeutralMode = lockMotor ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        turnMotorConfig.Feedback.SensorToMechanismRatio = GIntakeConstants.SensorToMechanismRatio;
        turnMotorConfig.Feedback.RotorToSensorRatio = GIntakeConstants.RotorToSensorRatio;

        turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.GIntakeConstants.Velocity;
        turnMotorConfig.MotionMagic.MotionMagicAcceleration = Constants.GIntakeConstants.Acceleration;
        turnMotorConfig.MotionMagic.MotionMagicJerk = Constants.GIntakeConstants.Jerk;

        /* Current Limiting */
        turnMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        turnMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        turnMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.02;
        return turnMotorConfig;
    }

    private TalonFXConfiguration getDriveMotorConfiguration() {
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* Current Limiting */
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        driveMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        driveMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.02;
        return driveMotorConfig;
    }

    public void setUpperIntakeControl(Runnable start, Runnable stop) {
        startUpperIntake = start;
        stopUppeerIntake = stop;
    }


    boolean isFirstTimeInit = true;
    public void init() {
        m_turnMotor.getConfigurator().apply(getTurnMotorConfiguration(true));
        m_driveMotor.getConfigurator().apply(getDriveMotorConfiguration());

        if (isFirstTimeInit) {
            isFirstTimeInit = false;
            if (!loadLastPosition()) {
                m_CANcoder.setPosition(0);
            }
        }
    }

    private String getFilePath() {
        if (RobotBase.isSimulation()) {
            return "d:/simulated_groundIntakeLastPosition.txt";
        } else {
            return "/home/lvuser/groundIntakeLastPosition.txt";
        }
    }

    private boolean loadLastPosition() {
        try {
            File file = new File(getFilePath());
            if (!file.exists()) {
                System.out.println("Turning arm saved canCode position file not found!");
                return false;
            }
            FileReader fileReader = new FileReader(file);
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            String line = bufferedReader.readLine();

            System.out.println("Turning arm load canCoder position succssfully. line is : " + line);
            bufferedReader.close();
            fileReader.close();
            m_CANcoder.setPosition(Double.parseDouble(line));

            return true;
        } catch (IOException e) {
            e.printStackTrace();
        }
        return false;
    }

    private void saveLastPosition() {
        try {
            File file = new File(getFilePath());
            if (!file.exists()) {
                file.createNewFile();
            }
            FileWriter fileWriter = new FileWriter(file);
            fileWriter.write(String.valueOf(m_CANcoder.getPosition().getValueAsDouble()));
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void onDisable() {
        saveLastPosition();
    }

    public void resetCancodePosition() {
        try {
            Files.delete(Path.of(getFilePath()));
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        m_CANcoder.setPosition(0);
    }


    public void unlockMotor() {
        m_turnMotor.getConfigurator().apply(getTurnMotorConfiguration(false));
    }

    public void lockMotor() {
        m_turnMotor.getConfigurator().apply(getTurnMotorConfiguration(true));
    }

    // public PositionVoltage goToRotationRequestion(double expectRotation) {
    //     PositionVoltage turnRequest = new PositionVoltage(expectRotation);
    //     SmartDashboard.putNumber("destination", expectRotation);
    //     return turnRequest;
    // }

    // public void expandGIntake() {
    //     // PositionVoltage request = goToRotationRequestion(GIntakeConstants.expandCCRotation);

    //     m_turnMotor.setControl(motionMagicVoltage.withPosition(GIntakeConstants.expandCCRotation).withSlot(0));
    // }

    // public void retractGIntake() {
    //     // PositionVoltage request = goToRotationRequestion(GIntakeConstants.retractCCRotation);
    //     // System.out.println("retract request");
    //     m_turnMotor.setControl(motionMagicVoltage.withPosition(GIntakeConstants.retractCCRotation).withSlot(0));
    // }

    public void expend() {
        if (curState == GI_STATE.IDLE) {
            curState = GI_STATE.EXPANDING;
        }
    }

    public void makeSureRetracted() {
        if (curState == GI_STATE.WAIT_FOR_CORAL) {
            curState = GI_STATE.IDLE;
        }
    }

    public void toggle() {
        switch (curState) {
            case IDLE:
                curState = GI_STATE.EXPANDING;
                break;
            case WAIT_FOR_CORAL:
                curState = GI_STATE.RETRACTING_WITHOUT_CORAL;
                break;
            default:
                break;
        }
    }

    public GI_STATE getState() {
        return curState;
    }

    public boolean isIdle() {
        return curState == GI_STATE.IDLE;
    }

    private boolean isArmAtPos(double targetPos) {
        if (Math.abs(m_CANcoder.getPosition().getValueAsDouble() - targetPos) < threshold) {
            return true;
        }

        return false;
    }

    public void zeroCC() {
        m_CANcoder.setPosition(0);
    }


    public boolean isCoralIn() {
        return getIsSensorOn();
    }

    protected void updateState() {
        double armPos = NONE_POS;
        switch (curState) {
            case IDLE:
                zeroCC();
                intakeMotorSpeed = 0;
                // armPos = 0;
                break;
            case EXPANDING:
                armPos = GIntakeConstants.expandCCRotation;
                intakeMotorSpeed = GIntakeConstants.intakeMotorSpeed;
                if (isArmAtPos(armPos)) {
                    curState = GI_STATE.WAIT_FOR_CORAL;
                }
                break;
            case WAIT_FOR_CORAL:
                intakeMotorSpeed = GIntakeConstants.intakeMotorSpeed;
                if (isCoralIn()) {
                    curState = GI_STATE.RETRACTING_WITH_CORAL;
                }
                break;
            case RETRACTING_WITH_CORAL:
                armPos = GIntakeConstants.retractCCRotation;
                intakeMotorSpeed = GIntakeConstants.intakeMotorSpeedInRetractingWithCoral;
                if (isArmAtPos(armPos)) {
                    intakeMotorSpeed = 0;
                    curState = GI_STATE.WAITING_FOR_REVERSE_INTAKE;
                    waitForReverseFrame = MAX_WAIT_FOR_REVERSE_FRAME;
                }
                break;
            case WAITING_FOR_REVERSE_INTAKE:
                intakeMotorSpeed = 0;
                if (waitForReverseFrame < 0) {
                    startUpperIntake.run();
                    curState = GI_STATE.REVERSE_INTAKING;
                    reverserIntakeCountdoownFrame = MAX_REVERSER_INTAKE_FRAME;
                }
                else {
                    waitForReverseFrame--;
                }
                break;
            case REVERSE_INTAKING:
                intakeMotorSpeed = GIntakeConstants.intakeMotorReverseSpeed;
                if (reverserIntakeCountdoownFrame < 0) {
                    curState = GI_STATE.IDLE;
                    intakeMotorSpeed = 0;
                }
                else {
                    reverserIntakeCountdoownFrame--;
                }
                break;
            case RETRACTING_WITHOUT_CORAL:
                armPos = GIntakeConstants.retractCCRotation;
                intakeMotorSpeed = 0;
                if (isArmAtPos(armPos)) {
                    curState = GI_STATE.IDLE;
                }
                break;
        }

        if (MiscUtils.compareDouble(armPos, NONE_POS)) {
            m_turnMotor.stopMotor();
        }
        else {
            m_turnMotor.setControl(motionMagicVoltage.withPosition(armPos).withSlot(0));
        }

        m_driveMotor.set(intakeMotorSpeed);

        SmartDashboard.putNumber("GI ccc targetPos", armPos);
        SmartDashboard.putNumber("GI ccc curPos", m_CANcoder.getPosition().getValueAsDouble());
        SmartDashboard.putString("GI ccc curState", curState.name());
        SmartDashboard.putNumber("GI ccc intakeMotorSpeed", intakeMotorSpeed);
    }

    // protected void updateState___OLD() {
    //     if (curState == GI_STATE.EXPAND) {
    //         startIntake();
    //     }
    //     if (curState == GI_STATE.NONE) {
    //         stopIntake();
    //     }
    //     if (curState == GI_STATE.RETRACT && curRunningState == GI_RUNNING_STATE.DONE) {
    //         // if (isCoralIn())
    //         // {
    //         //     reverseIntake();
    //         // }
    //         // else {
    //         //     stopIntake();
    //         // }
    //         zeroCC();
    //         // reverseIntake();
    //     }
    //     if (isCoralIn() && curState == GI_STATE.RETRACT && curRunningState == GI_RUNNING_STATE.RUNNING) {
    //         intakeMotorSpeed = 6;
    //     }

    //     do 
    //     {
    //         if (isCoralIn()) {
    //             if (curState == GI_STATE.EXPAND) {
    //                 delayTime = DELAY_FRAME;
    //                 setState(GI_STATE.RETRACT);
    //                 // stopIntake();
    //                 break;
    //             }
    //             // if (curState == GI_STATE.RETRACT && curRunningState == GI_RUNNING_STATE.RUNNING) {
    //             //     stopIntake();
    //             // }
    //             if (curState == GI_STATE.RETRACT && curRunningState == GI_RUNNING_STATE.DONE) {
    //                 startUpperIntake.run();
    //                 if (delayTime <= 0) {
    //                     reverseIntake();
    //                 }
    //                 else {
    //                     stopIntake();
    //                 }
    //                 delayTime--;
    //             }
    //         }
    //     } while(false);


    //     double pos = getStatePos(curState);
    //     SmartDashboard.putNumber("GI ccc targetPos", pos);
    //     SmartDashboard.putNumber("GI ccc curPos", m_CANcoder.getPosition().getValueAsDouble());
    //     SmartDashboard.putString("GI ccc runningState", curRunningState.name());
    //     SmartDashboard.putString("GI ccc curState", curState.name());
        
    //     if (MiscUtils.compareDouble(pos, NONE_POS)) {
    //         return;
    //     }
    //     if (isArmAtPos(pos)) {
    //         curRunningState = GI_RUNNING_STATE.DONE;
    //         // if (curState == GI_STATE.RETRACT && delayRevereIntakeFrame < 0) {
    //         //     stopIntake();
    //         // }
    //     }


    //     if (curRunningState == GI_RUNNING_STATE.RUNNING) {
    //         m_turnMotor.setControl(motionMagicVoltage.withPosition(pos));
    //     }
    //     else {
    //         m_turnMotor.stopMotor();
    //     }
        
    //     if (delayRevereIntakeFrame > 0) {
    //         delayRevereIntakeFrame--;
    //         if (delayRevereIntakeFrame == 0) {
    //             delayRevereIntakeFrame = -1;
    //             intakeMotorSpeed = 0;
    //         }
    //     }
    //     m_driveMotor.set(intakeMotorSpeed);
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateState();
        telemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    // public double getStatePos(GI_STATE state) {
    //     switch (state) {
    //         case EXPAND:
    //             return GIntakeConstants.expandCCRotation;
    //         case RETRACT:
    //             return GIntakeConstants.retractCCRotation;

    //         default:
    //             return NONE_POS;
    //     }
    // }

    public void toggleDebugSensor() {
        debugSensored = !debugSensored;
    }

    private boolean getIsSensorOn() {
        if (isDebugSensorOn) {
            return debugSensored;
        }
        return coralSensor.get() == false;
    }
    protected void telemetry() {
    
        SmartDashboard.putString("GI ccc sensor", "state: " + coralSensor.get());
        
    }
}
