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
    private boolean triggerCase = true; // If `update()` is called with this value, `isTrigger` will be set to `true`

    public ButtonManager() {}

    public ButtonManager(boolean b) {
        value = b;
        lastFrame = b;
    }

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
        if(value == triggerCase && !isTrigger){
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
     * Sets the case for when the internal trigger is fired. This is {@code true} by default, meaning the trigger responds to {@code update()} being called with {@code true} as the argument.
     * @param tCase
     */
    public void setTriggerCase(boolean tCase){
        this.triggerCase = tCase;
    }

    /**
     * @return whether we're currently on the first frame that the trigger has been activated
     */
    public boolean isTriggerRisingEdge(){
        return this.isTriggerRisingEdge;
    }
}
