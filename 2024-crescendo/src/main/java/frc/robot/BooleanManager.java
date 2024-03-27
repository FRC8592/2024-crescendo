package frc.robot;

public class BooleanManager {
    private boolean value = false;
    private boolean lastFrame = false;
    private boolean isRisingEdge = false;
    private boolean isFallingEdge = false;
    private boolean isToggle = false;
    private boolean lastToggle = false;
    private boolean isToggleRisingEdge = false;
    private boolean isToggleFallingEdge = false;
    private boolean isTriggered = false;

    public BooleanManager() {}
    
    public BooleanManager(boolean b) {
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

        if(isRisingEdge){
            isTriggered = true;
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
    public boolean isToggle() {
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
     * @return whether the internal trigger is true. This is the case when there has been a rising edge anytime since the last `resetTrigger()`
     */
    public boolean isTriggered(){
        return this.isTriggered;
    }

    public void resetTrigger(){
        this.isTriggered = false;
    }
}
