package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TEST", group = "TeleOp")
public class TEST extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor br = hardwareMap.dcMotor.get("backRight");
        DcMotor fr = hardwareMap.dcMotor.get("frontRight");

        br.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                br.setPower(1);
                fr.setPower(1);
            } else if (gamepad1.b) {
                br.setPower(-1);
                fr.setPower(-1);
            }
            else{
                br.setPower(0);
                fr.setPower(0);
            }
        }
    }
}
