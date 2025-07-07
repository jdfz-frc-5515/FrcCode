// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GIntakeConstants;
import frc.robot.subsystems.TurningArm2025.TurningArm2025.TA_STATE;
import frc.robot.utils.MiscUtils;

public class GroundIntakeSubsystem extends SubsystemBase {
    public enum GI_STATE {
        NONE,
        RETRACT,
        EXPAND,
        OUT_TAKE,   // 吐给上层Intake
    }

    public enum GI_RUNNING_STATE {
        READY,
        RUNNING,
        DONE,
    }

    private final TalonFX m_turnMotor = new TalonFX(GIntakeConstants.GIntakeTurnID);
    private final TalonFX m_driveMotor = new TalonFX(GIntakeConstants.GIntakeDriveID);
    private final CANcoder m_CANcoder = new CANcoder(GIntakeConstants.GIntakeCCID);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private final double NONE_POS = -9999;
    private final double threshold = 0.05;

    GI_STATE curState = GI_STATE.NONE;
    GI_RUNNING_STATE curRunningState = GI_RUNNING_STATE.READY;

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

        turnMotorConfig.Feedback.SensorToMechanismRatio = 1;
        turnMotorConfig.Feedback.RotorToSensorRatio = 16;

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

    public void toggle() {
        // if (curState == GI_STATE.EXPAND) {
        //     setState(GI_STATE.RETRACT);
        // } else if (curState == GI_STATE.RETRACT) {
        //     setState(GI_STATE.EXPAND);
        // } else {
        // }

        switch (curState) {
            case EXPAND:
                setState(GI_STATE.RETRACT);
                break;
            case RETRACT:
            case NONE:
            setState(GI_STATE.EXPAND);
                break;
            case OUT_TAKE:
                // out take does not change state
                break;
            default:
                break;
        }
    }

    public void setState(GI_STATE stat) {
        if (curState == stat) {
            return;
        }
        curState = stat;
        curRunningState = GI_RUNNING_STATE.RUNNING;
    }

    public GI_STATE getState() {
        return curState;
    }

    public GI_RUNNING_STATE getCurRunningState() {
        return curRunningState;
    }

    private boolean isDone(double targetPos) {
        if (Math.abs(m_CANcoder.getPosition().getValueAsDouble() - targetPos) < threshold) {
            return true;
        }

        return false;
    }

    public void zeroCC() {
        m_CANcoder.setPosition(0);
    }

    public void startIntake() {
        m_driveMotor.set(0.3);
    }

    public void stopIntake() {
        m_driveMotor.set(0);
    }

    public void reverseIntake() {
        m_driveMotor.set(-0.3);
    }


    protected void updateState() {
        if (curState == GI_STATE.EXPAND) {
            startIntake();
        }
        if (curState == GI_STATE.NONE) {
            stopIntake();
        }
        double pos = getStatePos(curState);
        SmartDashboard.putNumber("GI ccc targetPos", pos);
        SmartDashboard.putNumber("GI ccc curPos", m_CANcoder.getPosition().getValueAsDouble());
        SmartDashboard.putString("GI ccc runningState", curRunningState.name());
        SmartDashboard.putString("GI ccc curState", curState.name());

        if (MiscUtils.compareDouble(pos, NONE_POS)) {
            return;
        }
        if (isDone(pos)) {
            curRunningState = GI_RUNNING_STATE.DONE;
            if (curState == GI_STATE.RETRACT) {
                stopIntake();
            }
        }


        if (curRunningState == GI_RUNNING_STATE.RUNNING) {
            m_turnMotor.setControl(motionMagicVoltage.withPosition(pos));
        }
        else {
            m_turnMotor.stopMotor();
        }
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateState();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public double getStatePos(GI_STATE state) {
        switch (state) {
            case EXPAND:
                return GIntakeConstants.expandCCRotation;
            case RETRACT:
                return GIntakeConstants.retractCCRotation;

            default:
                return NONE_POS;
        }
    }
}
