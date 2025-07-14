// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import java.util.ArrayList;
import java.util.List;

public class PlayCandleSubsystem extends SubsystemBase {
  public CANdle candle = new CANdle(Constants.Candle.candleID, Constants.Candle.canBusName);
  public CANdleConfiguration config = new CANdleConfiguration();
  public RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 76, false, 8);
  public ColorFlowAnimation flowAnim = new ColorFlowAnimation(0, 255, 255, 1, 0.1, 76, ColorFlowAnimation.Direction.Backward, 8);
  public FireAnimation fireAnim = new FireAnimation(1, 0.5, 76, 0.5, 0.2, true, 8);
  private List<Animation> animationList = new ArrayList<>(3);
  private int anim_index = 0;
  // config.brightnessScalar = 0.25;
  // candle.configAllSettings(config);
  // RainbowAnimation anim = new RainbowAnimation(1, 0.5, 76, false, 8);
  // ColorFlowAnimation flow_anim = new ColorFlowAnimation(0, 255, 255, 1, 0.1, 76, ColorFlowAnimation.Direction.Backward, 8);
  // FireAnimation fire_anim = new FireAnimation(1, 0.5, 76, 0.5, 0.2, true, 8);
  // candle.animate(fire_anim);
  /** Creates a new ExampleSubsystem. */
  public PlayCandleSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void initCandle() {
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.25;
    candle.configAllSettings(config);
    animationList.add(rainbowAnim);
    animationList.add(flowAnim);
    animationList.add(fireAnim);
  }
  public void switchCandleAnim(int num) {
    candle.animate(animationList.get(num%3));
  }
  public void previousAnim() {
    anim_index -= 1;
    switchCandleAnim(anim_index);
  }
  public void nextAnim() {
    anim_index += 1;
    switchCandleAnim(anim_index);
  }
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
