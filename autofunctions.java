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
import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.GyroSensor;

public class autofunctions
{
    private DcMotor motorL_Up;
    private DcMotor motorL_Down;
    private DcMotor motorR_Up;
    private DcMotor motorR_Down;


    private Servo RedServo;
    private Servo BlackServo;

    private DcMotor ArmMotor_Left;
    private DcMotor ArmMotor_Right;

    private Servo armservo;

    private Servo shake_shack_servo;

    int CurrentPosition = 0;

    int ArmPosition = 0;


    private ElapsedTime runtime = new ElapsedTime();

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .169, correction;
    double     FORWARD_SPEED = 0.6;
    double     TURN_SPEED    = 0.5;

    BNO055IMU imu;

    Orientation angles;

    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;

    Telemetry telemetry;

    public void Initialize(DcMotor motorL_DownIn,
                           DcMotor motorR_DownIn,
                           DcMotor motorR_UpIn,
                           DcMotor motorL_UpIn,
                           Servo RedServoIn,
                           Servo BlackServoIn,
                           DcMotor ArmMotor_LeftIn,
                           DcMotor ArmMotor_RightIn,
                           Servo ArmServoIn,
                           Servo shake_shack_servoIn,
                           BNO055IMU imuIn,
                           NormalizedColorSensor colorSensor1In,
                           NormalizedColorSensor colorSensor2In,

                           Telemetry telemetryIn)
    {
        motorL_Down = motorL_DownIn;
        motorR_Down = motorR_DownIn;
        motorR_Up = motorR_UpIn;
        motorL_Up = motorL_UpIn;

        RedServo = RedServoIn;
        BlackServo = BlackServoIn;
        ArmMotor_Left = ArmMotor_LeftIn;
        ArmMotor_Right = ArmMotor_RightIn;
        armservo = ArmServoIn;
        shake_shack_servo = shake_shack_servoIn;
        imu = imuIn;
        colorSensor1 = colorSensor1In;
        colorSensor2 = colorSensor2In;

        telemetry = telemetryIn;


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

    public void ArmUpDown    (double Power, int Distance)
    {


        ArmMotor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmMotor_Left.setTargetPosition(-Distance);
        ArmMotor_Right.setTargetPosition(Distance);

        ArmMotor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmMotor_Left.setPower(-Power);
        ArmMotor_Right.setPower(Power);

        while (ArmMotor_Left.isBusy() && ArmMotor_Right.isBusy())
        {
            //Wait until the task is done
        }

        StopDriving();
    }

    public void ArmUpDownTime    (double Power, int Distance, int Time)
    {
        ArmMotor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmMotor_Left.setTargetPosition(-Distance);
        ArmMotor_Right.setTargetPosition(Distance);

        ArmMotor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmMotor_Left.setPower(-Power);
        ArmMotor_Right.setPower(Power);

        sleep(Time);

        StopDriving();
    }


   public void CloseServo ()
   {
       armservo.setPosition(1);
   }

   public void OpenServo ()
   {
       armservo.setPosition(0);
   }

    public int StrafeRightColor (double Power, int Distance)
    {

        int CurrentPosition = 0;

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
            int got_color = getcubecolor();
            if (got_color == 1)
            {
                CurrentPosition = motorR_Up.getCurrentPosition();
                break;
            }


        }

        StopDriving();

        return CurrentPosition;

    }

    public int StrafeLeftColor (double Power, int Distance)
    {

        int CurrentPosition = 0;

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
                CurrentPosition = motorR_Up.getCurrentPosition();
                break;
            }


        }

        StopDriving();

        return CurrentPosition;

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

        ArmMotor_Right.setPower(0);
        ArmMotor_Right.setPower(0);

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

    public void getcube()
    {

        int Distance = 200;
        //Drive Backward To move arm up
        DriveForward(0.469, 650);
        sleep(250);

        //Bring Arm Up
        ArmUpDown(0.7, 2200);
        sleep(250);

        //Drive Forward
        DriveForward(0.469,-600);
        sleep(569);

        //Close the Servo
        CloseServo();
        sleep(569);

        ArmUpDown(1,-900);

        int got_turn = getTurn();
        if (got_turn == 1)
        {
            Distance += 0;
        }
        if (got_turn == 2)
        {
            Distance -= 50;
        }
        if (got_turn == 3)
        {
            Distance -= 150;
        }

        DriveForward(0.469, Distance);

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

        if (Color.red(color) < 100 && Color.red(color) > 0 && Color.red(color2) < 100 && Color.red(color2) > 0) {
            telemetry.addLine("Got black");
            telemetry.update();
            return 1; // 1 = true
        }

        telemetry.addLine("nope");
        telemetry.update();
        return 0;  // 0 = false
    }

    public int getTurn()
    {
        if (CurrentPosition < 250 )
        {
            return 1;
        }
        if (CurrentPosition > 650)
        {
            return 3;
        }

            return 2;
    }

    public void TurnRightCompensation (double Power, int Distance)
    {
        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        int got_turn = getTurn();
        if (got_turn == 1)
        {
            Distance  += 25;
        }
        if (got_turn == 2)
        {
            Distance += 32  ;
        }
        if (got_turn == 3)
        {
            Distance += 69;
        }

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

    public void TurnLeftCompensation (double Power, int Distance)
    {
        motorR_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL_Down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AllBRAKE();

        int got_turn = getTurn();
        if (got_turn == 1)
        {
            Distance  += 25;
        }
        if (got_turn == 2)
        {
            Distance += 32  ;
        }
        if (got_turn == 3)
        {
            Distance += 69;
        }

        telemetry.addData("turndistance %d", Distance);
        telemetry.update();

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

        }

        StopDriving();
    }

    public void bluud_3_comp(int Distance)
    {
        ArmMotor_Right.getCurrentPosition();

        //Setting Arm Position to the Current Position
         ArmMotor_Right.getCurrentPosition();

        //Drives to first cube
        DriveForward(0.469, -1675);
        sleep(250);

        //Strafes so that color sensors are centered to the first cube
        StrafeLeft(0.1869,225);

        sleep(350);

        //Sets the Current Position function to the amount of encoder ticks it took for the color sensor to find black while strafing
        CurrentPosition = StrafeLeftColor(0.269, 2750);
        if(CurrentPosition < 0)
            CurrentPosition = CurrentPosition * -1;
        sleep(250);

        ArmMotor_Right.getCurrentPosition();

        getcube();
        sleep(100);

        //Turns Left to make a straight trajectory towards the foundation
        TurnLeftCompensation(0.4,1150);
        sleep(100);

        ArmUpDown(0.9,150);


        //Drives Forward for -1500 - Current Position towards the foundation
        telemetry.addData("CurrentPosition", "%d", CurrentPosition);
        telemetry.update();
        DriveForward(0.9, -2800-CurrentPosition+Distance);

    }

    public void bloodred_3_comp(int Distance)
    {
        ArmMotor_Right.getCurrentPosition();

        //Setting Arm Position to the Current Position
        ArmMotor_Right.getCurrentPosition();

        //Drives to first cube
        DriveForward(0.469, -1675);
        sleep(250);

        //Strafes so that color sensors are centered to the first cube
        StrafeRight(0.1869,225);

        //Sets the Current Position function to the amount of encoder ticks it took for the color sensor to find black while strafing
        CurrentPosition = StrafeRightColor(0.269, 2750);
        if(CurrentPosition < 0)
            CurrentPosition = CurrentPosition * -1;
        sleep(250);

        ArmMotor_Right.getCurrentPosition();

        //See autofunctions.java
        getcube();
        sleep(100);

        //Turns Left to make a straight trajectory towards the foundation
        TurnRightCompensation(0.4,1150);
        sleep(100);

        ArmUpDown(0.9,150);


        //Drives Forward for -1500 - Current Position towards the foundation
        telemetry.addData("CurrentPosition", "%d", CurrentPosition);
        telemetry.update();
        DriveForward(0.9, -2800-CurrentPosition+Distance);

    }
}











