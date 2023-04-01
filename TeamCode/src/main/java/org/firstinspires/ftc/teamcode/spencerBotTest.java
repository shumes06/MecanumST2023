package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class spencerBotTest extends LinearOpMode{
    private DcMotor rightFront, rightBack, leftFront, leftBack;
    private IMU imu;
    @Override
    public void runOpMode(){
    motorSetup();
    imuSetup();
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

    public void imuSetup(){
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
    public void drivePower(){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotIn = gamepad1.left_stick_x;
        double translY = -gamepad1.left_stick_y * 1.1;
        double translX = gamepad1.right_stick_x;
        double scaleInsure = Math.max(Math.abs(translY) + Math.abs(rotIn) + Math.abs(translX), 1);

        double rotTranslX = translX * Math.cos(-botHeading) - translY * Math.sin(-botHeading);
        double rotTranslY = translX * Math.sin(-botHeading) + translY * Math.cos(-botHeading);

        double leftFrontPower = (rotTranslY + rotIn + rotTranslX)/scaleInsure;
        double leftBackPower = (rotTranslY - rotIn + rotTranslX)/scaleInsure;
        double rightFrontPower = (rotTranslY - rotIn - rotTranslX)/scaleInsure;
        double rightBackPower = (rotTranslY + rotIn - rotTranslX)/scaleInsure;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);

        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }
    public void operation(){
        while(opModeIsActive()){
            drivePower();
            if (gamepad1.options) {
                imu.resetYaw();
            }
            telemetry.addData("Running?", opModeIsActive());
            telemetry.update();
        }
        telemetry.addData("Running?", opModeIsActive());
        telemetry.update();
    }
}
