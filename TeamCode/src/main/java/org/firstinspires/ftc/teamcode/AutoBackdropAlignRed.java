package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Backdrop Align Red", group ="A_Top")
//@Disabled
public class AutoBackdropAlignRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Backdrop Align",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
