package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Backdrop Wall Red", group ="A_Top")
//@Disabled
public class AutoBackdropWallRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Backdrop Park Wall Spin",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
