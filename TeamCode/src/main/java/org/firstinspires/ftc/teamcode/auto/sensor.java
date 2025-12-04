package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class sensor  extends OpMode {
    NormalizedColorSensor sen;

    public enum DetectedColor {
        RED,
        GREEN,
        BLUE
    }
    @Override
    public void init() {
        sen = hardwareMap.get(NormalizedColorSensor.class, "sensor");
    }
//    public DetectedColor getDetectedColor(Telemetry telemetry){
//        NormalizedRGBA colors = sen.getNormalizedColors();
//
//        float normRed, normGreen,normBlue;
//        normRed = colors.red / colors.alpha;
//        normGreen = colors.green / colors.alpha;
//        normBlue = colors.blue / colors.alpha;
//
//        telemetry.addData("red",normRed);
//        telemetry.addData("green",normGreen);
//        telemetry.addData("blue",normBlue);
//         //TODO add if statmetns for specif colors added
//        return DetectedColor.BLUE;
//    }
    @Override
    public void loop() {
        NormalizedRGBA colors = sen.getNormalizedColors();
        float normRed, normGreen,normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red",normRed);
        telemetry.addData("green",normGreen);
        telemetry.addData("blue",normBlue);
    }
}
