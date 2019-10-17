package org.firstinspires.ftc.teamcode.newprograms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import android.graphics.Color;
import android.os.SystemClock;

import static android.graphics.Color.WHITE;
import static android.graphics.Color.YELLOW;

public class autofunctions
{
    private DcMotor motorL_Up;
    private DcMotor motorL_Down;
    private DcMotor motorR_Up;
    private DcMotor motorR_Down;

    private Servo RedServo;
    private Servo BlackServo;

    NormalizedColorSensor colorSensor;
    NormalizedColorSensor colorSensor2;

    Telemetry telemetry;

    public void Initialize(DcMotor motorL_DownIn,
                           DcMotor motorR_DownIn,
                           DcMotor motorR_UpIn,
                           DcMotor motorL_UpIn,
                           Servo RedServoIn,
                           Servo BlackServoIn,

                           Telemetry telemetryIn)
    {
        motorL_Down = motorL_DownIn;
        motorR_Down = motorR_DownIn;
        motorR_Up = motorR_UpIn;
        motorL_Up = motorL_UpIn;
        RedServo = RedServoIn;
        BlackServo = BlackServoIn;
        
        telemetry = telemetryIn;


    }

    public void DriveForward(double Power, int Distance)
    {
        AllFLOAT();

        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorL_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(Distance);
        motorR_Up.setTargetPosition(-Distance);
        motorR_Down.setTargetPosition(-Distance);

        motorL_Down.setPower(Power);
        motorL_Up.setPower(Power);
        motorR_Up.setPower(-Power);
        motorR_Down.setPower(-Power);


        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }
        StopDriving();

        motorR_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public int DriveForwardcolor(double Power, int Distance)
    {
        AllFLOAT();

        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorL_Down.setTargetPosition(-Distance);

        motorR_Up.setPower(Power);
        motorR_Down.setPower(Power);
        motorL_Up.setPower(-Power);
        motorL_Down.setPower(-Power);


        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            int got_color = getcubecolor();
            if(got_color==1)
            {
                break;
            }

        }

        StopDriving();

        int CurrentPosition = motorR_Up.getCurrentPosition();

        motorR_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return CurrentPosition;
    }

    public void TurnLeft(double Power, int Distance)
    {
        AllFLOAT();

        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(Distance);
        motorL_Down.setTargetPosition(Distance);

        motorR_Up.setPower(Power);
        motorR_Down.setPower(Power);
        motorL_Up.setPower(Power);
        motorL_Down.setPower(Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }



    public void TurnRight(double Power, int Distance)
    {
        AllFLOAT();
        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorR_Up.setTargetPosition(-Distance);
        motorR_Down.setTargetPosition(-Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorL_Down.setTargetPosition(-Distance);

        motorR_Up.setPower(-Power);
        motorR_Down.setPower(-Power);
        motorL_Up.setPower(-Power);
        motorL_Down.setPower(-Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }

    public void StrafeRight(double Power, int Distance)
    {
        AllFLOAT();

        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorR_Up.setTargetPosition(-Distance);
        motorR_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorL_Down.setTargetPosition(Distance);

        motorR_Up.setPower(-Power);
        motorR_Down.setPower(Power);
        motorL_Up.setPower(-Power);
        motorL_Down.setPower(Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }

    public void StrafeLeft(double Power, int Distance)
    {
        AllFLOAT();

        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(-Distance);
        motorL_Up.setTargetPosition(Distance);
        motorL_Down.setTargetPosition(-Distance);

        motorR_Up.setPower(Power);
        motorR_Down.setPower(-Power);
        motorL_Up.setPower(Power);
        motorL_Down.setPower(-Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }

    public void StopDriving ()
    {
        motorL_Up.setPower(0);
        motorL_Down.setPower(0);
        motorR_Up.setPower(0);
        motorR_Down.setPower(0);

    }
    public int getcubecolor ()
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
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
                .addData("r", "%d", Color.red(color))
                .addData("g", "%d", Color.green(color))
                .addData("b", "%d", Color.blue(color));

        if (Color.red(color) < 150 && Color.red(color) > 115)
        {
            telemetry.addLine("Got Yellow");
            telemetry.update();
            return 1; // 1 = true
        }

        if (Color.red(color2) < 150 && Color.red(color2) > 115 )
        {
            telemetry.addLine("Got Yellow2");
            telemetry.update();
            return 1; // 1 = true
        }

        telemetry.addLine("nope");
        telemetry.update();
        return 0;  // 0 = false

    }

    public void AllFLOAT ()
    {
        motorR_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void AllBREAK ()
    {
        motorR_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void DriveForward_mrf(double Power, int Distance)
    {
        AllFLOAT();

        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorR_Up.setTargetPosition(Distance);


        motorR_Up.setPower(Power);



        while (motorR_Up.isBusy())
        {
            //Wait until the task is done
        }
        StopDriving();

        motorR_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveForward_mLup(double Power, int Distance)
    {
        AllFLOAT();

        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorL_Up.setTargetPosition(Distance);


        motorL_Up.setPower(Power);



        while (motorL_Up.isBusy())
        {
            //Wait until the task is done
        }
        StopDriving();

        motorL_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveForward_mRDown(double Power, int Distance)
    {
        AllFLOAT();

        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorR_Down.setTargetPosition(Distance);


        motorR_Down.setPower(Power);



        while (motorR_Down.isBusy())
        {
            //Wait until the task is done
        }
        StopDriving();

        motorR_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveForward_mLdown(double Power, int Distance)
    {
        AllFLOAT();

        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorL_Down.setTargetPosition(Distance);


        motorL_Down.setPower(Power);



        while (motorL_Down.isBusy())
        {
            //Wait until the task is done
        }
        StopDriving();

        motorL_Down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

}











