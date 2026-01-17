package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "ColorTest")
public class ColorTest extends OpMode {

    DcMotor spindexer;

    @Override
    public void init() {
        spindexer = hardwareMap.get(DcMotor.class, "spindexer");
    }


    @Override
    public void loop() {
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("spin",spindexer.getCurrentPosition());
        telemetry.update();
    }
}
