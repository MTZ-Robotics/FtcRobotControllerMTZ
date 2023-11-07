package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="Sample Park", group ="A_Top")
//@Disabled
public class AutoSamplePark extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {

        try {
            super.autoPaths("Red","SamplePark",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
