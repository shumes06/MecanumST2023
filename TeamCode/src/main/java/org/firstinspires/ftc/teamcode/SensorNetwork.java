package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SensorNetwork {
    
    private HardwareMap hardwareMap;
    private NormalizedColorSensor forwardColor;
    private NormalizedRGBA forwardColors;
    private LinearOpMode opModeTool;
    
    /* CLASS CONSTRUCTOR */
    public SensorNetwork(HardwareMap hardwareMap, LinearOpMode opModeTool) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
    } 
    
    public void initialize(float gainForward, float gainFront, float gainRear) {
        forwardColor = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
        forwardColor.setGain(gainForward);
    }
    
    public NormalizedRGBA getForwardColors() {
        forwardColors = forwardColor.getNormalizedColors();
        opModeTool.idle();
        return forwardColors;
    }
    
    public void getColors() {
        forwardColors = forwardColor.getNormalizedColors();
    }
    
}  // end class