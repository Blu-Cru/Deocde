package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

public class SinglePressGamepad {

    //logging only single presses for toggles

    //a,b,x,y in that order;
    private boolean[] lastFaceButtons;

    //dpad up down left right in that order;
    private boolean[] lastDpad;

    //bumpers, left then right;
    private boolean[] lastBumper;

    //stick buttons, left then right;
    private boolean[] lastStickButton;

    //options and share, in that order;
    private boolean[] lastOptionsAndShare;

    //triggers, left and right in that order;
    private boolean[] lastTriggersPressed;

    //touchpad
    private boolean lastTouchpad;

    //a,b,x,y in that order;
    private boolean[] thisFaceButtons;

    //dpad up down left right in that order;
    private boolean[] thisDpad;

    //bumpers, left then right;
    private boolean[] thisBumper;

    //stick buttons, left then right;
    private boolean[] thisStickButton;

    //options and share, in that order;
    private boolean[] thisOptionsAndShare;

    //triggers, left and right in that order;
    private boolean[] thisTriggersPressed;

    //touchpad
    private boolean thisTouchpad;

    Gamepad gamepad;
    double triggerTolerance = 0.2;

    public SinglePressGamepad(Gamepad gamepad){
        this.gamepad = gamepad;
        lastFaceButtons = new boolean[]{false, false, false, false};
        lastDpad = new boolean[]{false, false, false, false};
        lastBumper = new boolean[]{false, false};
        lastStickButton = new boolean[]{false, false};
        lastOptionsAndShare = new boolean[]{false, false};
        lastTriggersPressed = new boolean[]{false, false};
        lastTouchpad = false;
        thisFaceButtons = new boolean[]{false, false, false, false};
        thisDpad = new boolean[]{false, false, false, false};
        thisBumper = new boolean[]{false, false};
        thisStickButton = new boolean[]{false, false};
        thisOptionsAndShare = new boolean[]{false, false};
        thisTriggersPressed = new boolean[]{false, false};
        thisTouchpad = false;
    }

    public void update(){
        if (gamepad.a ){
            //pressed last loop
            thisFaceButtons[0] = !lastFaceButtons[0];
            lastFaceButtons[0] = true;
        } else {
            thisFaceButtons[0] = false;
            lastFaceButtons[0] = false;
        }

        if (gamepad.b){
            //pressed last loop
            thisFaceButtons[1] = !lastFaceButtons[1];
            lastFaceButtons[1] = true;
        } else {
            thisFaceButtons[1] = false;
            lastFaceButtons[1] = false;
        }

        if (gamepad.x){
            //pressed last loop
            thisFaceButtons[2] = !lastFaceButtons[2];
            lastFaceButtons[2] = true;
        } else {
            thisFaceButtons[2] = false;
            lastFaceButtons[2] = false;
        }

        if (gamepad.y){
            //pressed last loop
            thisFaceButtons[3] = !lastFaceButtons[3];
            lastFaceButtons[3] = true;
        } else {
            thisFaceButtons[3] = false;
            lastFaceButtons[3] = false;
        }



        if (gamepad.dpad_up){
            //pressed last loop
            thisDpad[0] = !lastDpad[0];
            lastDpad[0] = true;
        } else {
            thisDpad[0] = false;
            lastDpad[0] = false;
        }

        if (gamepad.dpad_down){
            //pressed last loop
            thisDpad[1] = !lastDpad[1];
            lastDpad[1] = true;
        } else {
            thisDpad[1] = false;
            lastDpad[1] = false;
        }

        if (gamepad.dpad_left){
            //pressed last loop
            thisDpad[2] = !lastDpad[2];
            lastDpad[2] = true;
        } else {
            thisDpad[2] = false;
            lastDpad[2] = false;
        }

        if (gamepad.dpad_right){
            //pressed last loop
            thisFaceButtons[3] = !lastDpad[3];
            lastDpad[3] = true;
        } else {
            thisDpad[3] = false;
            lastDpad[3] = false;
        }



        if (gamepad.left_bumper){
            //pressed last loop
            thisBumper[0] = !lastBumper[0];
            lastBumper[0] = true;
        } else {
            thisBumper[0] = false;
            lastBumper[0] = false;
        }

        if (gamepad.right_bumper){
            //pressed last loop
            thisBumper[1] = !lastBumper[1];
            lastBumper[1] = true;
        } else {
            thisBumper[1] = false;
            lastBumper[1] = false;
        }



        if (gamepad.left_stick_button){
            //pressed last loop
            thisStickButton[0] = !lastStickButton[0];
            lastStickButton[0] = true;
        } else {
            thisStickButton[0] = false;
            lastStickButton[0] = false;
        }

        if (gamepad.right_stick_button){
            //pressed last loop
            thisStickButton[1] = !lastStickButton[1];
            lastStickButton[1] = true;
        } else {
            thisStickButton[1] = false;
            lastStickButton[1] = false;
        }



        if (gamepad.options){
            //pressed last loop
            thisOptionsAndShare[0] = !lastOptionsAndShare[0];
            lastOptionsAndShare[0] = true;
        } else {
            thisOptionsAndShare[0] = false;
            lastOptionsAndShare[0] = false;
        }

        if (gamepad.share){
            //pressed last loop
            thisOptionsAndShare[1] = !lastOptionsAndShare[1];
            lastOptionsAndShare[1] = true;
        } else {
            thisOptionsAndShare[1] = false;
            lastOptionsAndShare[1] = false;
        }



        boolean ltPressed = gamepad.left_trigger > triggerTolerance;
        boolean rtPressed = gamepad.right_trigger > triggerTolerance;
        if (ltPressed){
            //pressed last loop
            thisTriggersPressed[0] = !lastTriggersPressed[0];
            lastTriggersPressed[0] = true;
        } else {
            thisTriggersPressed[0] = false;
            lastTriggersPressed[0] = false;
        }

        if (rtPressed){
            //pressed last loop
            thisTriggersPressed[1] = !lastTriggersPressed[1];
            lastTriggersPressed[1] = true;
        } else {
            thisTriggersPressed[1] = false;
            lastTriggersPressed[1] = false;
        }



        if (gamepad.touchpad){
            thisTouchpad = !lastTouchpad;
            lastTouchpad = true;
        } else {
            thisTouchpad = false;
            lastTouchpad = false;
        }
    }

    //face button functions
    public boolean pressedA(){
        return thisFaceButtons[0];
    }
    public boolean pressedB(){
        return thisFaceButtons[1];
    }
    public boolean pressedX(){
        return thisFaceButtons[2];
    }
    public boolean pressedY(){
        return thisFaceButtons[3];
    }

    //dpad functions
    public boolean pressedDpadUp(){
        return thisDpad[0];
    }
    public boolean pressedDpadDown(){
        return thisDpad[1];
    }
    public boolean pressedDpadLeft(){
        return thisDpad[2];
    }
    public boolean pressedDpadRight(){
        return thisDpad[3];
    }

    //bumper functions
    public boolean pressedLeftBumper(){
        return thisBumper[0];
    }
    public boolean pressedRightBumper(){
        return thisBumper[1];
    }

    //stick button functions
    public boolean pressedLeftStickButton(){
        return thisStickButton[0];
    }
    public boolean pressedRightStickButton(){
        return thisStickButton[1];
    }

    //options and share functions
    public boolean pressedOptions(){
        return thisOptionsAndShare[0];
    }
    public boolean pressedShare(){
        return thisOptionsAndShare[1];
    }

    //pressing triggers
    public boolean pressedLeftTrigger(){
        return thisTriggersPressed[0];
    }
    public boolean pressedRightTrigger(){
        return thisTriggersPressed[1];
    }

    //touchpad functions
    public boolean pressedTouchpad(){
        return thisTouchpad;
    }
}
