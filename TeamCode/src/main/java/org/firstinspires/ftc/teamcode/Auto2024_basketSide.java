package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="auto2024 Basket Side", group ="A_Top")
//@Disabled
public class Auto2024_basketSide extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {
        try {
            //Blue will reverse the turns and strafes
            super.autoPaths("Blue","auto2024",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
