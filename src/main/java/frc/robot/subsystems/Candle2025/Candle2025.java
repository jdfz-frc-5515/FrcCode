package frc.robot.subsystems.Candle2025;


// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.REVLibError;
// import com.revrobotics.RelativeEncoder;
// import edu.wpi.first.wpilibj.DigitalInput;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.List;

import org.opencv.core.Mat.Tuple2;
import org.opencv.core.Mat.Tuple3;

import com.ctre.phoenix.led.*;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.CANdle.VBatOutputMode;
// import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
// import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
// import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
// import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import frc.robot.Constants;

public class Candle2025 extends SubsystemBase {

    CANdle led = new CANdle(Constants.Candle.candleID, Constants.Candle.canBusName);

    private final int ledOffset = 8;
    private final int ledNum = 77;
    private final int slotNum = 2;
    private final Tuple2<Integer> rightPart = new Tuple2<Integer>(0, 25);
    private final Tuple2<Integer> leftPart = new Tuple2<Integer>(50, 76);
    enum COLOR {
        NONE,
        IDLE,
        WAIT_GROUND_CORAL,
        WAIT_UP_CORAL,
        BALL_MODE,
        CARRYING_CORAL,
        L1,
        L2,
        L3,
        L4,
        INTAKING,
        NUM,
    }

    private COLOR[][] slots = new COLOR[slotNum][ledNum];
    private COLOR[] mergedSlot = new COLOR[ledNum];
    
    private final List<Tuple3<Integer>> palette = Arrays.asList(
        new Tuple3<>(0, 0, 0),  // NONE
        new Tuple3<>(0, 0, 0),  // IDLE
        new Tuple3<>(0, 0, 0),  // WAIT_GROUND_CORAL
        new Tuple3<>(0, 0, 0),     // WAIT_UP_CORAL
        new Tuple3<>(15, 79, 242),      // BALL_MODE
        new Tuple3<>(245, 242, 66),     // CARRYING_CORAL
        new Tuple3<>(255, 255, 255),    // L1
        new Tuple3<>(235, 16, 16),      // L2
        new Tuple3<>(10, 247, 18),      // L3
        new Tuple3<>(168, 7, 222),       // L4
        new Tuple3<>(0, 0, 255)         // UP_INTAKING     
    );

    private boolean isNeedUpdate = false;

    public Candle2025() {
        led.setLEDs(0, 0, 0);
        clear();
    }


    public void clear() {
        for (int i = 0; i < slotNum; ++i) {
            clearSlot(i);
        }
    }
    
    // int count = 0;
    private void show() {
        // count++;
        // if (count % 100 == 0) {
        //     for (int i = 0; i < slotNum; ++i) {
        //         COLOR[] slot = slots[i];
        //         System.out.print("slot" + i + ": ");
        //         for (int j = 0; j < slot.length; ++j) {
        //             System.out.print(" " + slot[j].name() + ",");
        //         }
        //         System.out.println("");
        //     }
        // }

        for (int i = 0; i < slotNum; ++i) {
            COLOR[] slot = slots[i];
            for (int j = 0; j < slot.length; ++j) {
                if (i == 0) {
                    mergedSlot[j] = slot[j];
                }
                else {
                    if (slot[j] != COLOR.NONE) {
                        mergedSlot[j] = slot[j];
                    }
                }
            }
        }
        {
            COLOR[] slot = mergedSlot;
            int start = 0;
            while (start < ledNum) {
                COLOR currentColor = slot[start];
                // if (currentColor == COLOR.NONE) {
                //     start++;
                //     continue;
                // }
                int end = start + 1;
                while (end < ledNum && slot[end] == currentColor) {
                    end++;
                }
                Tuple3<Integer> color = palette.get(currentColor.ordinal());
                led.setLEDs(color.get_0().intValue(), color.get_1().intValue(), color.get_2().intValue(), 0,
                        start + ledOffset, end - start);
                start = end;
            }
        }
        // led.setLEDs(255, 255, 0);
    }

    public void showIdle() {
        int slotIdx = 0;
        COLOR[] slot = slots[slotIdx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.IDLE;
        }

        isNeedUpdate = true;
    }

    public void showIntaking(boolean isShow) {
        if (isShow) {
            int slotIdx = 0;
            COLOR[] slot = slots[slotIdx];
            for (int i = 0; i < slot.length; ++i) {
                slot[i] = COLOR.INTAKING;
            }
            isNeedUpdate = true;            
        }
        else {
            clearSlot(0);
        }
    }

    public void showCarryingCoral() {
        int slotIdx = 0;
        COLOR[] slot = slots[slotIdx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.CARRYING_CORAL;
        }
        isNeedUpdate = true;
    }

    public void showWaitUpCoral() {
        int slotIdx = 0;
        COLOR[] slot = slots[slotIdx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.WAIT_UP_CORAL;
        }
        isNeedUpdate = true;
    }

    public void showWaitGroundCoral() {
        int slotIdx = 0;
        COLOR[] slot = slots[slotIdx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.WAIT_GROUND_CORAL;
        }
        isNeedUpdate = true;
    }

    public void showBallMode() {
        int slotIdx = 0;
        COLOR[] slot = slots[slotIdx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.BALL_MODE;
        }
        isNeedUpdate = true;
    }

    public void showL1() {
        clearLn();
        // int slotIdx = 1;
        // COLOR[] slot = slots[slotIdx];
        showLn(COLOR.L1, true);
        showLn(COLOR.L1, false);
        // for (int i = 0; i < slot.length; ++i) {
        //     slot[i] = COLOR.L1;
        // }
        isNeedUpdate = true;
    }

    private void showLn(COLOR c, boolean isLeft) {
        Tuple2<Integer> part = isLeft ? leftPart : rightPart;
        int slotIdx = 1;
        COLOR[] slot = slots[slotIdx];
        for (int i = part.get_0(); i <= part.get_1(); ++i) {
            slot[i] = c;
        }
        isNeedUpdate = true;
    }

    public void showL2() {
        clearLn();
        showLn(COLOR.L2, true);
        showLn(COLOR.L2, false);
    }

    public void showL3() {
        clearLn();
        showLn(COLOR.L3, true);
        showLn(COLOR.L3, false);
    }

    public void showL4() {
        clearLn();
        showLn(COLOR.L4, true);
        showLn(COLOR.L4, false);
    }

    public void showSpecial(boolean isShow) {
        if (!isShow) {
            clearSlot(1);
            return;
        }
        int slotIdx = 1;
        COLOR[] slot = slots[slotIdx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.L4;
        }
        isNeedUpdate = true;
    }

    private void clearSlot(int idx) {
        if (idx < 0 || idx >= slotNum) {
            return;
        }
        COLOR[] slot = slots[idx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.NONE;
        }
        isNeedUpdate = true;
    }


    public void clearLn() {
        clearSlot(1);
    }


    private void update() {
        if (isNeedUpdate)
        {
            show();
            isNeedUpdate = false;
        }
        
    }

    @Override
    public void periodic() {
        update();
    }

    public void init() {
        showIdle();
    }

    public void onDisable() {
        clear();
    }
}
