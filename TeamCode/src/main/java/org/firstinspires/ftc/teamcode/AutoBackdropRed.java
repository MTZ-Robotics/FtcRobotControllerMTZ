package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Backdrop Red", group ="A_Top")
//@Disabled
public class AutoBackdropRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Backdrop",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
