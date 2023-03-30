package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class spencerBotTest extends LinearOpMode{
    private DcMotor rightFront, rightBack, leftFront, leftBack;

    @Override
    public void runOpMode(){
    motorSetup();
    waitForStart();
    operation();
    }

    public void motorSetup(){
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void drivePower(){
        double translX = gamepad1.left_stick_x;
        double translY = -gamepad1.left_stick_y;
        double rot = gamepad1.right_stick_x;

        leftFront.setPower(translY + rot + translX);
        leftBack.setPower(translY - rot + translX);

        rightFront.setPower(translY - rot - translX);
        rightBack.setPower(translY + rot - translX);
    }
    public void operation(){
        while(opModeIsActive()){
            drivePower();
            telemetry.addData("Running?", opModeIsActive());
            telemetry.update();
        }
        telemetry.addData("Running?", opModeIsActive());
        telemetry.update();
    }
}
