// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GIntakeConstants;

public class GroundIntakeSubsystem extends SubsystemBase {
    private final TalonFX m_turnMotor = new TalonFX(GIntakeConstants.GIntakeTurnID);
    private final TalonFX m_driveMotor = new TalonFX(GIntakeConstants.GIntakeDriveID);
    private final CANcoder m_CANcoder = new CANcoder(GIntakeConstants.GIntakeCCID);

    private TalonFXConfiguration turnMotorConfiguration() {
        TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
        turnMotorConfig.Slot0.kP = GIntakeConstants.turnMotorConst.kP;
        turnMotorConfig.Slot0.kI = GIntakeConstants.turnMotorConst.kI;
        turnMotorConfig.Slot0.kD = GIntakeConstants.turnMotorConst.kD;
        turnMotorConfig.Slot0.kS = GIntakeConstants.turnMotorConst.kS;
        turnMotorConfig.Slot0.kV = GIntakeConstants.turnMotorConst.kV;
        turnMotorConfig.Slot0.kA = GIntakeConstants.turnMotorConst.kA;
        turnMotorConfig.Feedback.FeedbackRemoteSensorID = GIntakeConstants.GIntakeCCID;
        turnMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return turnMotorConfig;
    }

    private TalonFXConfiguration driveMotorConfiguration() {
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        return driveMotorConfig;
    }

    public void configGroundIntake() {
        m_turnMotor.getConfigurator().apply(turnMotorConfiguration());
        m_driveMotor.getConfigurator().apply(driveMotorConfiguration());
    }

    public PositionVoltage goToRotationRequestion(double expectRotation) {
        PositionVoltage turnRequest = new PositionVoltage(expectRotation);
        SmartDashboard.putNumber("destination", expectRotation);
        return turnRequest;
    }

    public void expandGIntake() {
        PositionVoltage request = goToRotationRequestion(GIntakeConstants.expandCCRotation);
        System.out.println("expand request");
        m_turnMotor.setControl(request);
    }

    public void retractGIntake() {
        PositionVoltage request = goToRotationRequestion(GIntakeConstants.retractCCRotation);
        System.out.println("retract request");
        m_turnMotor.setControl(request);
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
        m_driveMotor.set(-0.1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
