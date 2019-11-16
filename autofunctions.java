package org.firstinspires.ftc.teamcode.newprograms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import android.graphics.Color;
import android.os.SystemClock;

import static android.graphics.Color.WHITE;
import static android.graphics.Color.YELLOW;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.GyroSensor;

public class autofunctions
{
    private DcMotor motorL_Up;
    private DcMotor motorL_Down;
    private DcMotor motorR_Up;
    private DcMotor motorR_Down;
    private DcMotor motor_UpDown;
    private DcMotor motor_SideSide;

    private Servo RedServo;
    private Servo BlackServo;

    private DcMotor ArmMotor_Left;
    private DcMotor ArmMotor_Right;

    private Servo armservo;


    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    Orientation angles;

    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;

    Telemetry telemetry;

    public void Initialize(DcMotor motorL_DownIn,
                           DcMotor motorR_DownIn,
                           DcMotor motorR_UpIn,
                           DcMotor motorL_UpIn,
                           DcMotor motor_UpDownIn,
                           DcMotor motor_SideSideIn,
                           Servo RedServoIn,
                           Servo BlackServoIn,
                           Servo ArmServoIn,
                           BNO055IMU imuIn,

                           Telemetry telemetryIn)
    {
        motorL_Down = motorL_DownIn;
        motorR_Down = motorR_DownIn;
        motorR_Up = motorR_UpIn;
        motorL_Up = motorL_UpIn;
        motor_UpDown = motor_UpDownIn;
        motor_SideSide = motor_SideSideIn;
        RedServo = RedServoIn;
        BlackServo = BlackServoIn;
        armservo = ArmServoIn;
        imu = imuIn;

        telemetry = telemetryIn;


    }

    public void SetMode()
    {

    }

    public void DriveForward(double Power, int Distance)
    {


        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorL_Down.setTargetPosition(-Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(Distance);

        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorL_Down.setPower(-Power);
        motorL_Up.setPower(-Power);
        motorR_Up.setPower(Power);
        motorR_Down.setPower(Power);


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

    public void TurnLeft(double Power, int Distance)
    {


        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(Distance);
        motorL_Down.setTargetPosition(Distance);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);



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

        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(-Distance);
        motorR_Down.setTargetPosition(-Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorL_Down.setTargetPosition(-Distance);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);



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


        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(-Distance);
        motorR_Down.setTargetPosition(Distance);
        motorL_Up.setTargetPosition(-Distance);
        motorL_Down.setTargetPosition(Distance);

        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);



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


        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(-Distance);
        motorL_Up.setTargetPosition(Distance);
        motorL_Down.setTargetPosition(-Distance);


        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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

    public void StrafeLeftkazar4 (double Power, int Distance)
    {


        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(-Distance);
        motorL_Up.setTargetPosition(Distance);
        motorL_Down.setTargetPosition(-Distance);


        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorR_Up.setPower(Power);
        motorR_Down.setPower(-Power);
        motorL_Up.setPower(Power);
        motorL_Down.setPower(-Power);

        while(motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading:", angles.firstAngle);
        telemetry.addData("Roll:", angles.secondAngle);
        telemetry.addData("Pitch:", angles.thirdAngle);
        telemetry.update();
        }

    }

    public void StrafeLeftColor (double Power, int Distance)
    {


        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        motorR_Up.setTargetPosition(Distance);
        motorR_Down.setTargetPosition(-Distance);
        motorL_Up.setTargetPosition(Distance);
        motorL_Down.setTargetPosition(-Distance);


        motorR_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL_Down.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorR_Up.setPower(Power);
        motorR_Down.setPower(-Power);
        motorL_Up.setPower(Power);
        motorL_Down.setPower(-Power);

        while (motorR_Up.isBusy() && motorR_Down.isBusy()&& motorL_Up.isBusy()&& motorL_Down.isBusy())
        {
            int got_color = getcubecolor();
            if (got_color == 1)
            {
                break;
            }
        }

        StopDriving();

    }


    public void ServoDown ()
    {
      BlackServo.setPosition(-0.8);
      RedServo.setPosition(-0.8);
    }

    public void ServoUp ()
    {
        BlackServo.setPosition(0.8);
        RedServo.setPosition(0.8);
    }

    public void StopDriving ()
    {
        motorL_Up.setPower(0);
        motorL_Down.setPower(0);
        motorR_Up.setPower(0);
        motorR_Down.setPower(0);

    }
    public void ArmLeftRight (double Power, int Distance)
    {
        motor_SideSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_SideSide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_SideSide.setTargetPosition(Distance);
        motor_SideSide.setPower(Power);
    }

    public void ArmUpDown    (double Power, int Distance)
    {
        motor_UpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_UpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_UpDown.setTargetPosition(Distance);
        motor_UpDown.setPower(Power);
    }

    public void AllFLOAT ()
    {
        motorR_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void AllBRAKE ()
    {
        motorR_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getcubecolor() {
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


        telemetry.addLine("normalized color 18: ")
                .addData("a", "%d", Color.alpha(color))
                .addData("r", "%d", Color.red(color))
                .addData("g", "%d", Color.green(color))
                .addData("b", "%d", Color.blue(color));

        if (Color.red(color) < 100 && Color.red(color) > 80) {
            telemetry.addLine("Got Yellow");
            telemetry.update();
            return 1; // 1 = true
        }

        if (Color.red(color2) < 100 && Color.red(color2) > 80) {
            telemetry.addLine("Got Yellow2");
            telemetry.update();
            return 2; // 2 = true
        }

        int got_color = getcubecolor();
        if (got_color == 1)
        {

        }

        telemetry.addLine("nope");
        telemetry.update();
        return 0;  // 0 = false
    }

}











