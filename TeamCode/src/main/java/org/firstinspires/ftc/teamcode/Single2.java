package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
@Disabled

public class Single2 extends LinearOpMode{
    DcMotor l,r;
    
    @Override
    public void runOpMode()
    {
        l = hardwareMap.get(DcMotor.class, "single");
        l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        r = hardwareMap.get(DcMotor.class, "single2");
        r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        waitForStart();
        
        while (opModeIsActive()) {
            l.setPower(gamepad1.right_stick_x);
            r.setPower(-gamepad1.right_stick_x);
        }

    }
}