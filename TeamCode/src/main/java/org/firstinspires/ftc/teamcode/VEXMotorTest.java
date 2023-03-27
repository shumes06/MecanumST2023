package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
@Disabled

public class VEXMotorTest extends LinearOpMode{
    private Servo single;
    
    @Override
    public void runOpMode()
    {
        single = hardwareMap.get(Servo.class, "single393");
        
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                single.setDirection(Servo.Direction.FORWARD);
                single.setPosition(0.75); //0.5-0.75
            } else if (gamepad1.dpad_down) {
                single.setDirection(Servo.Direction.REVERSE);
                single.setPosition(0.75);
            } 
            else{
                single.setPosition(0);
            }
        }
    }
}