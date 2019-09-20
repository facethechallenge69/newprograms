package org.firstinspires.ftc.teamcode.newprograms;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "ColorTele", group = "Tutorials")
public class NEWNEW_colortele extends LinearOpMode {
    NormalizedColorSensor colorSensor;



    @Override
    public void runOpMode() throws InterruptedException {


        colorSensor = (NormalizedColorSensor) hardwareMap.colorSensor.get("colorSensor");


        waitForStart();

        while (opModeIsActive()) {

            NormalizedRGBA colors1 = colorSensor.getNormalizedColors();


            int color = colors1.toColor();


            float max = Math.max(Math.max(Math.max(colors1.red, colors1.green), colors1.blue), colors1.alpha);
            colors1.red /= max;
            colors1.green /= max;
            colors1.blue /= max;
            color = colors1.toColor();


            telemetry.addLine("normalized color 18:  ")
                    .addData("a", "%d", Color.alpha(color))
                    .addData("r1", "%d", Color.red(color));


            telemetry.update();
        }
    }
}



