package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "ColorTest")
public class ColorTest extends OpMode {

    NormalizedColorSensor sen;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    @Override
    public void init() {
        sen = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
    }

    public DetectedColor getDetectedColor(NormalizedRGBA colors) {

        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        // GREEN detection (dominant green)
        if (g > 0.40 && g > r && g > b) {
            return DetectedColor.GREEN;
        }

        // PURPLE detection (red + blue strong, green weak)
        if (r > 0.30 && b > 0.30 && g < 0.25) {
            return DetectedColor.PURPLE;
        }

        return DetectedColor.UNKNOWN;
    }

    @Override
    public void loop() {
        NormalizedRGBA colors = sen.getNormalizedColors();

        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        telemetry.addData("R", r);
        telemetry.addData("G", g);
        telemetry.addData("B", b);

        DetectedColor detected = getDetectedColor(colors);
        telemetry.addData("Detected", detected);

        telemetry.update();
    }
}
