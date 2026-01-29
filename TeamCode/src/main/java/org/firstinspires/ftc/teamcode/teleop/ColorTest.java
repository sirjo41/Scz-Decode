package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Colortest")
public class ColorTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        Servo ss = hardwareMap.get(Servo.class,"feeder");
        intakeSensor.setGain(2);
        ss.setPosition(1);
        telemetry.addLine("sdjfsdfsdfjsd");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                ss.setPosition(0.5);
            }
            else{
                ss.setPosition(1);
            }
            NormalizedRGBA colors = intakeSensor.getNormalizedColors();
            telemetry.addData("Green",colors.green);
            telemetry.addData("Red",colors.red);
            telemetry.addData("Blue",colors.blue);
            telemetry.update();

        }
    }
}
