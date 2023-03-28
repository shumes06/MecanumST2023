import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Core;
import org.opencv.imgproc.Moments;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.ArrayList;

@TeleOp

public class WebcamColorRead extends LinearOpMode{
    
    OpenCvWebcam webcam;
    
    public static int x;
    public static int y;
    
    public static int minH = 0;
    public static int minS = 0;
    public static int minV = 0;
    
    public static int maxH = 255;
    public static int maxS = 255;
    public static int maxV = 255;
    
    public static void delay(long ms)
    {
        try {
            TimeUnit.MILLISECONDS.sleep(ms);
        }
        catch(InterruptedException e) {
            
        }
    }
    
    void webcamSetup() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        webcam.setPipeline(new colorPipe2());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                
            }
        });
    }
    
    @Override
    public void runOpMode() {
        webcamSetup();
        
        waitForStart();
        
        int val = 0;
        int count = 0;
        
        //code ends here
        while(opModeIsActive())
        {
            count += 1;
            if (val == 0) {
                telemetry.addLine("minH");
                if (gamepad1.right_bumper) {
                    minH += 1;
                }
                if (gamepad1.left_bumper) {
                    minH -= 1;
                }
            }
            
            if (val == 1) {
                telemetry.addLine("minS");
                if (gamepad1.right_bumper) {
                    minS += 1;
                }
                if (gamepad1.left_bumper) {
                    minS -= 1;
                }
            }
            
            if (val == 2) {
                telemetry.addLine("minV");
                if (gamepad1.right_bumper) {
                    minV += 1;
                }
                if (gamepad1.left_bumper) {
                    minV -= 1;
                }
            }
            
            if (val == 3) {
                telemetry.addLine("maxH");
                if (gamepad1.right_bumper) {
                    maxH += 1;
                }
                if (gamepad1.left_bumper) {
                    maxH -= 1;
                }
            }
            
            if (val == 4) {
                telemetry.addLine("maxS");
                if (gamepad1.right_bumper) {
                    maxS += 1;
                }
                if (gamepad1.left_bumper) {
                    maxS -= 1;
                }
            }
            
            if (val == 5) {
                telemetry.addLine("maxV");
                if (gamepad1.right_bumper) {
                    maxV += 1;
                }
                if (gamepad1.left_bumper) {
                    maxV -= 1;
                }
            }
            
            telemetry.addData("MinH", minH);
            telemetry.addData("MinS", minS);
            telemetry.addData("MinV", minV);
            telemetry.addData("MaxH", maxH);
            telemetry.addData("MaxH", maxS);
            telemetry.addData("MaxH", maxV);
            
            telemetry.update();
            
            if(gamepad1.dpad_up && count % 5 == 0) {
                val += 1;
                if (val >= 6) val = 0;
            }
            
            delay(50);
        }
    }
}

class colorPipe2 extends OpenCvPipeline {
    Mat blurredImage = new Mat();
    Mat hsvImage = new Mat();
    
    Mat thresh = new Mat();
    
    Mat output = new Mat();
    
    Mat dilated = new Mat();
    Mat hierarchey = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    
    @Override 
    public Mat processFrame(Mat input)
    {
        contours.clear();
        
        output = input;
        
        Imgproc.blur(input, blurredImage, new Size(3, 3));
        Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
        
        Core.inRange(hsvImage, new Scalar(WebcamColorRead.minH, WebcamColorRead.minS, WebcamColorRead.minV),
        new Scalar(WebcamColorRead.maxH, WebcamColorRead.maxS, WebcamColorRead.maxV), thresh);     
        
        return thresh;
        
        /*
        //dilate the image, to make it better for the next step
        //make the blobs bigger
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.dilate(thresh, dilated, kernel);
        
        //find the contours of the image
        Imgproc.findContours(dilated, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 1);
        
        double maxVal = 0;
        int maxValIdx = 0;
        if (!contours.isEmpty()) {
            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
            {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                if (maxVal < contourArea)
                {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }
        } 
        
        //make sure the contour is big enough. if not, put some text on screen and return.
        //this means that the correct color is detected
        if (maxVal < 100) 
        {
            Imgproc.putText(output, "no target", new Point(10, 20), 0, 1, new Scalar(0, 0, 255));
            return output;
        }
        
        Imgproc.drawContours(input, contours, maxValIdx, new Scalar(255,0,0), 2);
        
        //find the center of the biggest contour
        Moments mu = Imgproc.moments(contours.get(maxValIdx));
        WebcamColorRead.x = (int) (mu.get_m10() / mu.get_m00());
        WebcamColorRead.y = (int) (mu.get_m01() / mu.get_m00());
        
        //draw a circle at the center of the contour, write where it is, and exit
        Imgproc.circle(output, new Point(WebcamColorRead.x, WebcamColorRead.y), 3, new Scalar(0,255,255));
        Imgproc.putText(output, "target at x = " + WebcamColorRead.x, new Point(10, 20), 1, 1, new Scalar(0, 0, 0));
        
        
        return output;
        */
        
    }
}

class color
{
    //
    static int green =      0x00ff00;
    
    //
    static int lightBlue =  0x00ffff;
    
    //
    static int darkBlue =   0x0000ff;
    
    //
    static int red =        0xff0000;
    
    //
    static int yellow =     0xffff00;
    
    //
    static int black =      0x000000;
    
    //
    static int white =      0xffffff;

    //static int lightGreen = 0xbada55;
    //static int darkGreen = 0x65535;
    //static int darkRed = 0x800000;
    //static int pink = 0xff80ed;
    //static int uglyYellow = 0x996515
}