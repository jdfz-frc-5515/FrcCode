package frc.robot.subsystems;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ImprovedCommandXboxController extends CommandXboxController {
    /** Represents a digital button on an XboxController. */
    public enum Button {
        kAutoButton(0),
        /** Left bumper. */
        kLeftBumper(5),
        /** Right bumper. */
        kRightBumper(6),
        /** Left stick. */
        kLeftStick(9),
        /** Right stick. */
        kRightStick(10),
        /** A. */
        kA(1),
        /** B. */
        kB(2),
        /** X. */
        kX(3),
        /** Y. */
        kY(4),
        /** Back. */
        kBack(7),
        /** Start. */
        kStart(8),
        /** Left trigger. */
        kLeftTrigger(15),
        /** Right trigger. */
        kRightTrigger(16);

        /** Button value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods. This is done by
         * stripping the leading `k`, and if not a Bumper button append `Button`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("Bumper")) {
                return name;
            }
            if (name.endsWith("Trigger")) {
                return name;
            }
            return name + "Button";
        }
    }
    private double m_triggerThreshold = 0.0;
    public ImprovedCommandXboxController(final int port) {
        super(port);
        m_triggerThreshold = 0.5;
        //TODO Auto-generated constructor stub
    }
    public ImprovedCommandXboxController(final int port, final double _triggerThreshold) {
        super(port);
        m_triggerThreshold = _triggerThreshold;
    }
    /**
     * Read the value of the left trigger (LT) button on the controller.
     *
     * @return The axis of the trigger is greater than the trigger threshold
     */
    public boolean getLeftTrigger(){
        return getLeftTriggerAxis() > m_triggerThreshold;
    }
    /**
     * Read the value of the right trigger (RT) button on the controller.
     *
     * @return The axis of the trigger is greater than the trigger threshold
     */
    public boolean getRightTrigger(){
        return getRightTriggerAxis() > m_triggerThreshold;
    }
    /**
     * Get the button value (starting at button 1).
     *
     * <p>The buttons are returned in a single 16 bit value with one bit representing the state of
     * each button. The appropriate button is returned as a boolean value.
     *
     * <p>This method returns true if the button is being held down at the time that this method is
     * being called.
     *
     * @param button The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getButton(int button){
        if(button==Button.kAutoButton.value) {
            return true;
        }
        if(button == Button.kLeftTrigger.value){
            return getLeftTrigger();
        }
        if(button == Button.kRightTrigger.value){
            return getRightTrigger();
        }
        return getHID().getRawButton(button);
    }

    public boolean getButton(Button button){
        return getButton(button.value);
    }

    /**
     * THIS DOES NOT APPLY TO AUTOBUTTON OR TRIGGERS!
     * @param button
     * @return
     */
    public boolean getButtonPressed(int button){
        return getHID().getRawButtonPressed(button);
    }

    /**
     * THIS DOES NOT APPLY TO TRIGGERS!
     * @param button
     * @return
     */
    public boolean getButtonPressed(Button button){
        return getButtonPressed(button.value);
    }

    // public ChassisSpeeds getDesiredRelativeSpeeds(){
    //     return new ChassisSpeeds(this.getLeftY(), this.getLeftX(), -this.getRightX() * 10.);
    // }
}
