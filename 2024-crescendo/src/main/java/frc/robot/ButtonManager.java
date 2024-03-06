package frc.robot;

public class ButtonManager {
    private boolean value = false;
    private boolean lastFrame = false;
    private boolean isRisingEdge = false;
    private boolean isFallingEdge = false;
    private boolean isToggle = false;
    private boolean lastToggle = false;
    private boolean isToggleRisingEdge = false;
    private boolean isToggleFallingEdge = false;
    private boolean isTrigger = false; // Locks onto `true` when `update()` is called with value `triggerCase`. Reset only with a method
    private boolean isTriggerRisingEdge = false;

    /**
     * Class containing logic to make managing buttons easier. You must call {@code update()} with the raw value of the button every robot frame. With that requirement satisfied, this class tracks the following: <ul>
     * <li> Whether the button is currently on a rising edge (in other words, whether this frame was called with {@code true} while the last was called with {@code false}).
     * <li> Whether the button is on a falling edge (similar to a rising edge, but {@code false} after {@code true}).
     * <li> A toggle, which starts at {@code false} and is flipped everytime a rising edge is triggered.
     * <li> Rising and falling edges for the toggle.
     * <li> A "trigger", which is a boolean that starts at {@code false} and is "locked" to {@code true} when {@code update(true)} is called. This can be reset with {@code resetTrigger()}.
     */
    public ButtonManager() {}

    public boolean isPressed() {
        return value;
    }

    public void update(boolean b) {
        value = b;
        isRisingEdge = value ? (lastFrame ? false : true) : false;
        isFallingEdge = value ? false : (lastFrame ? true : false);
        lastFrame = b;

        if (isRisingEdge) {
            isToggle = !isToggle;
        }
        isToggleRisingEdge = isToggle ? (lastToggle ? false : true) : false;
        isToggleFallingEdge = isToggle ? false : (lastToggle ? true : false);
        lastToggle = isToggle;

        isTriggerRisingEdge = false;
        if(value && !isTrigger){
            isTrigger = true;
            isTriggerRisingEdge = true;
        }
    }

    /**
     * @return a boolean representing whether the last {@code update()} was {@code true} while the one before it was {@code false}.
     */
    public boolean isRisingEdge() {
        return this.isRisingEdge;
    }

    /**
     * @return a boolean representing whether the last {@code update()} was {@code false} while the one before it was {@code true}.
     */
    public boolean isFallingEdge() {
        return this.isFallingEdge;
    }

    /**
     * @return a boolean representing the state of the internal toggle (flipped by a rising edge)
     */
    public boolean isToggled() {
        return this.isToggle;
    }
    
    /**
     * @return a boolean representing whether the internal toggle was {@code false} last {@code update()} and is {@code true} now
     */
    public boolean isToggleRisingEdge() {
        return this.isToggleRisingEdge;
    }

    /**
     * @return a boolean representing whether the internal toggle was {@code true} last {@code update()} and is {@code false} now
     */
    public boolean isToggleFallingEdge() {
        return this.isToggleFallingEdge;
    }

    /**
     * @param isToggle boolean to manually set the internal toggle to. Bypasses rising and falling edge detection.
     */
    public void setToggle(boolean isToggle){
        this.isToggle = isToggle;
        this.lastToggle = isToggle;
    }

    /**
     * @return {@code true} if {@code update(triggerCase)} (see {@code setTriggerCase()}) has been called since the last {@code resetTrigger()}
     */
    public boolean isTriggered(){
        return this.isTrigger;
    }

    /**
     * Reset the internal trigger
     */
    public void resetTrigger(){
        this.isTrigger = false;
    }

    /**
     * @return whether we're currently on the first frame that the trigger has been activated
     */
    public boolean isTriggerRisingEdge(){
        return this.isTriggerRisingEdge;
    }
}
