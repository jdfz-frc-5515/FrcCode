package frc.robot.subsystems.Intake2025;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MiscUtils;

public class Intake2025 extends SubsystemBase {
    public static final TalonFX m_motor = new TalonFX(Constants.Intake.motorID,
            Constants.Intake.canBusName);
    private VelocityVoltage intakeVelDutycycle = new VelocityVoltage(0);

    private DigitalInput intakeCoralSensor0 = new DigitalInput(0);      // upper sensor
    private DigitalInput intakeCoralSensor1 = new DigitalInput(1);      // lower sensor
    private int intakeCoralSensorOnTickCount = -1;
    private int intakeCoralSensorOffTickCount = -1;
    private final int intakeCoralSensorOnTickCountThreshold = 0;       // a tick is about 1/50 second(50Hz)
    private final int intakeCoralSensorOffTickCountThreshold = 0;      // a tick is about 1/50 second(50Hz)

    private boolean canAdjusetCoralPosition = false;

    public enum STATE {
        READY,
        CORAL_IN,
        GOT_CORAL,
        CARRYING_CORAL,
        CORAL_OUT,
        BALL_IN,
        BALL_OUT,
        CARRYING_BALL,
    }

    private STATE curState = STATE.READY;

    public Intake2025() {
    }

    private TalonFXConfiguration getMotorConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLowerLimit = 30;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.02;

        
        /* Open and Closed Loop Ramping */
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;


        config.Slot0.kP = Constants.Intake.KP;
        config.Slot0.kI = Constants.Intake.KI;
        config.Slot0.kD = Constants.Intake.KD;
        config.Slot0.kS = Constants.Intake.KS;
        config.Slot0.kV = Constants.Intake.KV;
        config.Slot0.kA = Constants.Intake.KA;

        return config;
    }

    public void init() {
        m_motor.getConfigurator().apply(getMotorConfiguration());
        setState(STATE.READY);
    }

    public void setCanAdjusetCoralPosition(boolean on) {
        canAdjusetCoralPosition = on;
    }

    public void toggleCoralIntake() {
        if (this.curState == STATE.READY) {
            setState(STATE.CORAL_IN);
        }
        else if (this.curState == STATE.CORAL_IN) {
            setState(STATE.READY);
        }
        else if (this.curState == STATE.CARRYING_CORAL) {
            setState(STATE.CORAL_OUT);
        }
        else {
            setState(STATE.READY);
        }
    }

    public void toggleBallIntake() {
        if (this.curState == STATE.READY) {
            setState(STATE.BALL_IN);
        }

        else if (this.curState == STATE.BALL_IN) {
            setState(STATE.READY);
        }
        else if (this.curState == STATE.CARRYING_BALL) {
            setState(STATE.BALL_OUT);
        }
        else {
            setState(STATE.READY);
        }
    }

    public void toggleBallIntake(boolean isOn) {
        if (this.curState == STATE.READY && isOn) {
            setState(STATE.BALL_IN);
        }
        else if (this.curState == STATE.BALL_IN && !isOn) {
            setState(STATE.READY);
        }
    }

    public void startIntake() {
        if (this.curState == STATE.READY) {
            setState(STATE.CORAL_IN);
        }
    }

    public void startShoot() {
        if (this.curState == STATE.CARRYING_CORAL) {
            canAdjusetCoralPosition = false;
            setState(STATE.CORAL_OUT);
        }
    }

    public void setState(STATE st) {
        this.curState = st;
    }

    public boolean getIsCarryingCoral() {
        // return isIntakeCoralSensorOn;
        // System.out.println("------------------------=============> " + curState.name());
        return curState == STATE.CARRYING_CORAL || curState == STATE.CORAL_OUT;
    }

    private boolean isAllSensored() {
        return !intakeCoralSensor0.get() && !intakeCoralSensor1.get();
    }

    private boolean isCoralTotallyOut() {
        return intakeCoralSensor0.get() == true && intakeCoralSensor1.get() == true;
    }


    private void updateIsCarryingCarol() {
        boolean isCarrying = !intakeCoralSensor0.get();
        if (isCarrying) {
            intakeCoralSensorOffTickCount = -1;
            if (intakeCoralSensorOnTickCount == -1) {
                intakeCoralSensorOnTickCount = 0;
            }
            else {
                intakeCoralSensorOnTickCount++;
            }
            if (intakeCoralSensorOnTickCount > intakeCoralSensorOnTickCountThreshold) {
            }
        }
        else {
            intakeCoralSensorOnTickCount = -1;
            if (intakeCoralSensorOffTickCount == -1) {
                intakeCoralSensorOffTickCount = 0;
            }
            else {
                intakeCoralSensorOffTickCount++;
            }
            if (intakeCoralSensorOffTickCount > intakeCoralSensorOffTickCountThreshold) {
            }
        }
    };

    enum CORAL_IN_STATE {
        READY,
        STEP1,
        STEP2,
    };

    CORAL_IN_STATE coralInState = CORAL_IN_STATE.READY;

    int delayFrame = 0;
    private double updateCoralIn() {
        double speed = 0;
        switch (coralInState) {
            case READY:
            {
                speed = Constants.Intake.coralInSpeed;
                if (!intakeCoralSensor0.get()) {
                    speed = Constants.Intake.coralInSlowSpeed;
                }
                if (isAllSensored()) {
                    // coralInState = CORAL_IN_STATE.STEP1;
                    // speed = Constants.Intake.coralInReverseSpeed;

                    // majun 2025-7-9:
                    
                    coralInState = CORAL_IN_STATE.STEP1;
                    delayFrame = 0;
                }
            }
            break;
            case STEP1:
            {
                speed = Constants.Intake.coralInSpeed;
                delayFrame--;
                if (delayFrame <= 0) {
                    speed = 0;
                    curState = STATE.CARRYING_CORAL;
                    coralInState = CORAL_IN_STATE.READY;
                }
                break;
            }
            // case STEP1:
            // {
            //     speed = Constants.Intake.coralInReverseSpeed;
            //     if (intakeCoralSensor1.get()) {
            //         // sensor1 not on
            //         curState = STATE.CARRYING_CORAL;
            //         coralInState = CORAL_IN_STATE.READY;
            //         speed = 0;
            //     }
            // }
            // break;
            case STEP2:
            {

            }
            break;
        }
        return speed;
    }
    private void updateState() {
        double speed = 0;
        switch (curState) {
            case READY:
                canAdjusetCoralPosition = false;
                break;
            case CARRYING_BALL:
                speed = 0;
                break;
            case CARRYING_CORAL:
                if (!intakeCoralSensor1.get() && canAdjusetCoralPosition) {
                    // Pose2d p = null;
                    // p.getX();
                    speed = Constants.Intake.coralInReverseSpeed;
                }
                else {
                    speed = 0;
                }
                break;
            case CORAL_IN:
                speed = updateCoralIn();
                break;
            // case GOT_CORAL:
            //     speed = -15;
            //     gotCoralTicks++;
            //     if (gotCoralTicks >= maxGotCoralTicks) {
            //         curState = STATE.CARRYING_CORAL;
            //     }
            //     break;
            case CORAL_OUT:
                if (isCoralTotallyOut()) {
                    curState = STATE.READY;
                }
                else {
                    speed = Constants.Intake.coralOutSpeed;
                }
                break;
            case BALL_IN:
                speed = Constants.Intake.BallInSpeed;
                break;
            case BALL_OUT:
                speed = Constants.Intake.BAllOutSpeed;
                break;
            default:
                break;
        }

        if (MiscUtils.compareDouble(speed, 0)) {
            m_motor.stopMotor();
        }
        else {
            intakeVelDutycycle.Velocity = speed;
            m_motor.setControl(intakeVelDutycycle);
        }
    }

    @Override
    public void periodic() {
        // updateIsCarryingCarol();
        updateState();
        telemetry();
    }

    protected void telemetry() {
        SmartDashboard.putString("Intake2025 state", curState.name());
        SmartDashboard.putString("Intake2025 coral in state", coralInState.name());
        SmartDashboard.putString("Intake2025_Sensor0", "state: " + intakeCoralSensor0.get());
        SmartDashboard.putString("Intake2025_Sensor1", "state: " + intakeCoralSensor1.get());

    }
}
