// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.TurningArm2025;

import java.text.BreakIterator;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.awt.Container;
import java.io.BufferedReader;
import java.io.FileReader;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator2025.Elevator2025.EV_STATE;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.SmartDashboardEx;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurningArm2025 extends SubsystemBase {
    public enum TA_STATE {
        NONE,
        ZERO,
        GI, // ground intake
        UI, // up intake
        L1,
        L2,
        L3,
        L4,
        BALL1,
        BALL2,
        DODGE,

        TUNING_UP,
        TUNING_DOWN,
    }

    public enum RUNNING_STATE {
        READY,
        RUNNING,
        DONE,
    }

    private final double NONE_POS = -9999;
    private final double threshold = 0.05;

    TA_STATE curState = TA_STATE.NONE;
    RUNNING_STATE curRunningState = RUNNING_STATE.READY;

    private String getFilePath() {
        if (RobotBase.isSimulation()) {
            return "d:/simulated_turningArmLastPosition.txt";
        } else {
            return "/home/lvuser/turningArmLastPosition.txt";
        }
    }

    /** Creates a new ExampleSubsystem. */
    public TurningArm2025() {
    }

    private final TalonFX m_armMotor = new TalonFX(Constants.TurningArm.motorID, Constants.TurningArm.canBusName);
    private final CANcoder m_canCoder = new CANcoder(Constants.TurningArm.canCoderID, Constants.TurningArm.canBusName);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    // private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    int runingCount = 0;
    @Override
    public void periodic() {
        // runingCount++;
        // // This method will be called once per scheduler run
        // SmartDashboard.putNumber("TurningArm run", runingCount);
        // SmartDashboard.putBoolean("Elevator Top", elevatorTop());
        updateState();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private TalonFXConfiguration getMotorConfiguration(boolean lockMotor) {
        TalonFXConfiguration elevatorConfiguration = new TalonFXConfiguration();
        elevatorConfiguration.MotorOutput.NeutralMode = lockMotor ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -200;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        elevatorConfiguration.Slot0.kP = Constants.TurningArm.KP;
        elevatorConfiguration.Slot0.kI = Constants.TurningArm.KI;
        elevatorConfiguration.Slot0.kD = Constants.TurningArm.KD;
        elevatorConfiguration.Slot0.kS = Constants.TurningArm.KS;
        elevatorConfiguration.Slot0.kV = Constants.TurningArm.KV;
        elevatorConfiguration.Slot0.kA = Constants.TurningArm.KA;

        elevatorConfiguration.MotionMagic.MotionMagicCruiseVelocity = Constants.TurningArm.Velocity;
        elevatorConfiguration.MotionMagic.MotionMagicAcceleration = Constants.TurningArm.Acceleration;
        elevatorConfiguration.MotionMagic.MotionMagicJerk = Constants.TurningArm.Jerk;
        
        elevatorConfiguration.Feedback.FeedbackRemoteSensorID = Constants.TurningArm.canCoderID;
        elevatorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        elevatorConfiguration.Feedback.SensorToMechanismRatio = Constants.TurningArm.SensorToMechanismRatio;
        elevatorConfiguration.Feedback.RotorToSensorRatio = Constants.TurningArm.RotorToSensorRatio;

        return elevatorConfiguration;
    }

    boolean isFirstTimeInit = true;
    public void init() {
        m_armMotor.getConfigurator().apply(getMotorConfiguration(true));
        if (isFirstTimeInit) {
            isFirstTimeInit = false;
            if (!loadLastPosition()) {
                m_canCoder.setPosition(0);
            }
        }

    }

    public void initInTestMode() {
        setState(TA_STATE.NONE);
        curRunningState = RUNNING_STATE.READY;
        m_armMotor.getConfigurator().apply(getMotorConfiguration(true));
        m_armMotor.stopMotor();
    }

    public void toggleTuningUp() {
        if (curState == TA_STATE.TUNING_UP) {
            setState(TA_STATE.NONE);
            m_armMotor.stopMotor();
        }
        else {
            setState(TA_STATE.TUNING_UP);
        }
    }

    public void toggleTuningDown() {
        if (curState == TA_STATE.TUNING_DOWN) {
            setState(TA_STATE.NONE);
            m_armMotor.stopMotor();
        }
        else {
            setState(TA_STATE.TUNING_DOWN);
        }
    }
    public void setState(TA_STATE stat) {
        if (curState == stat) {
            return;
        }
        curState = stat;
        curRunningState = RUNNING_STATE.RUNNING;
    }

    public TA_STATE getState() {
        return curState;
    }

    public RUNNING_STATE getCurRunningState() {
        return curRunningState;
    }

    private boolean isDone(double targetPos) {
        if (Math.abs(m_canCoder.getPosition().getValueAsDouble() - targetPos) < threshold) {
            return true;
        }

        return false;
    }

    protected void updateState() {
        SmartDashboardEx.putNumberSDOnly("ARM ccc1", m_canCoder.getPosition().getValueAsDouble());
        SmartDashboardEx.putNumberSDOnly("ARM ccc2", m_armMotor.getPosition().getValueAsDouble());

        double output = 0.030;
        if (curState == TA_STATE.TUNING_UP) {
            // driveVelocity.Acceleration = 0.1;
            // driveVelocity.Velocity = 0.05;
            // m_armMotor.setControl(driveVelocity);
            driveDutyCycle.Output = output;
            m_armMotor.setControl(driveDutyCycle);
            SmartDashboardEx.putNumberSDOnly("ELEVATOR ccc TUNING pos", m_canCoder.getPosition().getValueAsDouble());
            curRunningState = RUNNING_STATE.DONE;
            return;
        } else if (curState == TA_STATE.TUNING_DOWN) {
            // driveVelocity.Acceleration = 0.1;
            // driveVelocity.Velocity = -0.05;
            // m_armMotor.setControl(driveVelocity.withVelocity(-0.011));

            driveDutyCycle.Output = -output;
            m_armMotor.setControl(driveDutyCycle);
            SmartDashboardEx.putNumberSDOnly("ELEVATOR ccc TUNING pos", m_canCoder.getPosition().getValueAsDouble());
            curRunningState = RUNNING_STATE.DONE;
            return;
        }
        
        double pos = getStatePos(curState);
        SmartDashboardEx.putNumber("ARM ccc targetPos", pos);
        SmartDashboardEx.putString("ARM ccc curState", curState.name());

        if (MiscUtils.compareDouble(pos, NONE_POS)) {
            return;
        }
        if (isDone(pos)) {
            curRunningState = RUNNING_STATE.DONE;
        }


        m_armMotor.setControl(motionMagicVoltage.withPosition(pos));
    }

    private void saveLastPosition() {
        try {
            File file = new File(getFilePath());
            if (!file.exists()) {
                file.createNewFile();
            }
            FileWriter fileWriter = new FileWriter(file);
            fileWriter.write(String.valueOf(m_canCoder.getPosition().getValueAsDouble()));
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
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
            m_canCoder.setPosition(Double.parseDouble(line));

            return true;
        } catch (IOException e) {
            e.printStackTrace();
        }
        return false;
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
        m_canCoder.setPosition(0);
    }

    public boolean getIsInCoralSafeZone() {
        // 所谓SafeZone是指在这个区间中，Intake可以启动Coral的位置校正，把持有的Coral往回吸一点，这个过程不会让Coral的尾部接触到漏斗的下边缘。
        double curPos = m_canCoder.getPosition().getValueAsDouble();
        if (curPos > Constants.TurningArm.upIntakePos && curPos < Constants.TurningArm.ball1Pos) {
            return true;
        }

        return false;
    }

    // int upC = 0;
    // int dC = 0;

    // public void elevatorUp() {
    //     upC++;
    //     SmartDashboard.putNumber("EUp", upC);
    //     // motionMagicVoltage.Position = Constants.TurningArm.Top;
    //     m_arm.setControl(motionMagicVoltage.withPosition(Constants.TurningArm.Top));
    // }

    // public void elevatorDown() {
    //     dC++;
    //     SmartDashboard.putNumber("EDown", dC);
    //     // motionMagicVoltage.Position = Constants.TurningArm.Bottom;
    //     m_arm.setControl(motionMagicVoltage.withPosition(Constants.TurningArm.Bottom));
    // }

    // public static boolean elevatorTop() {
    //     if (m_arm.getPosition().getValueAsDouble() == Constants.TurningArm.Top)
    //         return true;
    //     else
    //         return false;
    // }

    public void unlockMotor() {
        m_armMotor.getConfigurator().apply(getMotorConfiguration(false));
    }

    public void lockMotor() {
        m_armMotor.getConfigurator().apply(getMotorConfiguration(true));
    }

    public double getStatePos(TA_STATE state) {
        switch (state) {
            case ZERO:
                return Constants.TurningArm.zeroPos;
            case GI:
                return Constants.TurningArm.groundIntakePos;
            case UI:
                return Constants.TurningArm.upIntakePos;
            case L1:
                return Constants.TurningArm.l1Pos;
            case L2:
                return Constants.TurningArm.l2Pos;
            case L3:
                return Constants.TurningArm.l3Pos;
            case L4:
                return Constants.TurningArm.l4Pos;
            case DODGE:
                return Constants.TurningArm.dodgePos;
            case BALL1:
                return Constants.TurningArm.ball1Pos;
            case BALL2:
                return Constants.TurningArm.ball2Pos;
            default:
                return NONE_POS;
        }
    }
}
