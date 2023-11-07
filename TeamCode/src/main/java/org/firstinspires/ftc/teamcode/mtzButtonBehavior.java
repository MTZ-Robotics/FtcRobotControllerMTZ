package org.firstinspires.ftc.teamcode;

public class mtzButtonBehavior {

    public boolean isUp, isDown, clickedUp, clickedDown;

    public void update(boolean buttonDown){
        if(buttonDown){
            if (this.isUp){
                this.clickedDown=true;
            } else {
                this.clickedDown=false;
            }
            this.isDown = true;
            this.isUp = false;
            this.clickedUp = false;
        } else {
            if (this.isDown){
                this.clickedUp=true;
            } else {
                this.clickedUp=false;
            }
            this.isUp = true;
            this.isDown = false;
            this.clickedDown = false;
        }
    }
}
