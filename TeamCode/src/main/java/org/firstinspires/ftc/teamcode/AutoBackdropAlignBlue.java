package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Backdrop Align Blue", group ="A_Top")
//@Disabled
public class AutoBackdropAlignBlue extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Blue","Backdrop Align",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
