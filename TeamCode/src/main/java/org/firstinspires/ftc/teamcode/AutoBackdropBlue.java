package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Backdrop Blue", group ="A_Top")
//@Disabled
public class AutoBackdropBlue extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Blue","Backdrop",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
