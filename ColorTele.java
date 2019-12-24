package org.firstinspires.ftc.teamcode.newprograms;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "colortele", group = "Tutorials")
public class ColorTele extends LinearOpMode {
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    TouchSensor touch;


    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor1 = (NormalizedColorSensor) hardwareMap.colorSensor.get("red_color");

        colorSensor2 = (NormalizedColorSensor) hardwareMap.colorSensor.get("black_color");

        touch =  hardwareMap.touchSensor.get("touch_sensor");

        waitForStart();

        while (opModeIsActive()) {

            NormalizedRGBA colors = colorSensor1.getNormalizedColors();
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();

            int color = colors.toColor();
            int color2 = colors2.toColor();

            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            float max2 = Math.max(Math.max(Math.max(colors2.red, colors2.green), colors2.blue), colors2.alpha);
            colors2.red /= max2;
            colors2.green /= max2;
            colors2.blue /= max2;
            color2 = colors2.toColor();



            telemetry.addLine("normalized color 18:  ")
                    .addData("a", "%d", Color.alpha(color))
                    .addData("r1", "%d", Color.red(color))
                    .addData("b1", "%d", Color.blue(color))
                    .addData("g1", "%d", Color.green(color))
                    .addData("a2","%d",Color.alpha(color2))
                    .addData("r2", "%d", Color.red(color2))
                    .addData("b2", "%d", Color.blue(color2))
                    .addData("g2","%d",Color.green(color2));



            if (Color.red(color) < 100 && Color.red(color) > 0) {
                telemetry.addLine("Got black");


            }

            if (Color.red(color2) < 100 && Color.red(color2) > 0) {
                telemetry.addLine("Got black2");


            }

            telemetry.addLine("nope");
            telemetry.update();
        }
    }
}


