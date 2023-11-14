package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Backdrop Sample Red", group ="A_Top")
//@Disabled
public class AutoBackdropSampleRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Backdrop Sample",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
