package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ColorTest")
public class ColorTest extends OpMode {

    Servo ss;
    @Override
    public void init() {
        ss = hardwareMap.get(Servo.class, "feeder");
        ss.setPosition(1);
    }

    @Override
    public

    void loop() {
        if (gamepad1.a) {
            ss.setPosition(0.55);
        } else if (gamepad1.b) {
            ss.setPosition(1);
        }

        telemetry.addData("spin", ss.getPosition());
        telemetry.update();
    }
}
