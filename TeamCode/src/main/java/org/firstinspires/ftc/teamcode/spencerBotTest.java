package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;

@Autonomous
@Disabled

public class spencerBotTest extends LinearOpMode{
    private Blinker control_Hub;
    public void blinkerSetup(){
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
    }
    public int color = 0;
    @Override
    public void runOpMode(){
        while (opModeIsActive()){
            control_Hub.setConstant(color);
            color +=1;
        }
    }
}
