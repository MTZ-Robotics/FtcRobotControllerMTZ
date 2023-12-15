package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Backdrop Align Wall Red", group ="A_Top")
//@Disabled
public class AutoBackdropAlignWallRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Backdrop Align Park Wall Spin",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
