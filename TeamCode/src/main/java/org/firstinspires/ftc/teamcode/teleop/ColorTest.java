package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeSensor.setGain(2);
        waitForStart();
        while (opModeIsActive()){
            NormalizedRGBA colors = intakeSensor.getNormalizedColors();
            telemetry.addData("Green",colors.green);
            telemetry.addData("Red",colors.red);
            telemetry.addData("Blue",colors.blue);

        }
    }
}
