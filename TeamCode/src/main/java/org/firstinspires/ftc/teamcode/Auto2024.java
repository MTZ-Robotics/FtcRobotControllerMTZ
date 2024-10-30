package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="2024", group ="A_Top")
//@Disabled
public class Auto2024 extends AutoControlsMTZ_PushBot {
    public void runOpMode() throws InterruptedException {
        try {
            super.autoPaths("Red","Auto2024",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
