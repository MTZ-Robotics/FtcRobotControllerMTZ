package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Audience Sample Red", group ="A_Top")
//@Disabled
public class AutoAudienceSampleRed extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","Audience Sample",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
