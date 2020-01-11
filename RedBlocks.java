package org.firstinspires.ftc.teamcode.newprograms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.newprograms.autofunctions;

@Autonomous(name = "RedBlocks", group = "Tutorials")
public class RedBlocks extends LinearOpMode
{
    private DcMotor motorL_Up;
    private DcMotor motorL_Down;
    private DcMotor motorR_Up;
    private DcMotor motorR_Down;


    private Servo RedServo;
    private Servo BlackServo;

    private DcMotor ArmMotor_Left;
    private DcMotor ArmMotor_Right;

    private Servo shake_shack_servo;

    private Servo armservo;

    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    Orientation angles;

    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;

    autofunctions auto_functions = new autofunctions();

    int CurrentPosition = 0;

    int ArmPosition = 0;



    //  Telemetry telemetry = new Telemetry();


    @Override
    public void runOpMode() throws InterruptedException {
        //Receiving the necessary hardware for the motors
        motorL_Down = hardwareMap.dcMotor.get("left_motor_d");
        motorR_Down = hardwareMap.dcMotor.get("right_motor_d");
        motorL_Up = hardwareMap.dcMotor.get("left_motor_up");
        motorR_Up = hardwareMap.dcMotor.get("right_motor_up");

        RedServo = hardwareMap.servo.get("red_servo");
        BlackServo = hardwareMap.servo.get("black_servo");
        shake_shack_servo = hardwareMap.servo.get("servo_arm");

        ArmMotor_Left = hardwareMap.dcMotor.get("armmotor_l");
        ArmMotor_Right = hardwareMap.dcMotor.get("armmotor_r");

        armservo = hardwareMap.servo.get("arm_servo");

        colorSensor1 = (NormalizedColorSensor) hardwareMap.colorSensor.get("red_color");
        colorSensor2 = (NormalizedColorSensor) hardwareMap.colorSensor.get("black_color");




        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int encoder_tics = 0;
        double encoder_speed = 0;

        //Setting the behavior for the motors to float.
        motorR_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmMotor_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        auto_functions.Initialize(motorL_Down,
                motorR_Down,
                motorR_Up,
                motorL_Up,
                RedServo,
                BlackServo,
                ArmMotor_Left,
                ArmMotor_Right,
                armservo,
                shake_shack_servo,
                imu,
                colorSensor1,
                colorSensor2,
                telemetry);

        //Init Position
        auto_functions.ServoUp();

        auto_functions.OpenServo();
        //Position should be that the foundation servos are up against the wall, and the gaps between
        //the wheel motors is centered along the line of mat.

        //Waiting for Start to be pressed

        ArmMotor_Right.getCurrentPosition();

        shake_shack_servo.setPosition(1);

        waitForStart();

        //Current Position of ArmMotor Right
        ArmMotor_Right.getCurrentPosition();

        //Setting Arm Position to the Current Position
        ArmPosition = ArmMotor_Right.getCurrentPosition();

        //Drives to first cube
        auto_functions.DriveForward(0.5, -1600);
        sleep(250);

        //Strafes so that color sensors are centered to the first cube
        auto_functions.StrafeRight(0.4,225);

        //Sets the Current Position function to the amount of encoder ticks it took for the color sensor to find black while strafing
        CurrentPosition = auto_functions.StrafeRightColor(0.2, 2750);
        if(CurrentPosition < 0)
            CurrentPosition = CurrentPosition * -1;
        sleep(250);

        ArmMotor_Right.getCurrentPosition();

        //See autofunctions.java
        auto_functions.getcube();
        sleep(100);

        //Turns Left to make a straight trajectory towards the foundation
        auto_functions.TurnRight(0.5,1200);
        sleep(100);


        //Drives Forward for -1500 - Current Position towards the foundation
        telemetry.addData("CurrentPosition", "%d", CurrentPosition);
        auto_functions.DriveForward(0.9, -3000-CurrentPosition);

        ArmMotor_Right.getCurrentPosition();


        sleep(100);

        //Setting Up for Sleep
        auto_functions.ArmUpDownTime(0.7, 400, 500);

        //Opening servo to drop cube
        auto_functions.OpenServo();

        sleep(100);

        auto_functions.ArmUpDown(0.8,-116);

        //Drives to next cube


        auto_functions.DriveForward(0.769, CurrentPosition+3769);


        auto_functions.TurnLeft(0.8,1100);

        ArmMotor_Right.getCurrentPosition();

        auto_functions.StrafeRight(0.7,769);

        auto_functions.DriveForward(0.8, -769);

        auto_functions.CloseServo();
        sleep(569);

        auto_functions.ArmUpDown(0.7,-400);

        auto_functions.DriveForward(0.7,769);

        auto_functions.TurnRight(0.8,1269);

        auto_functions.DriveForward(0.8,-CurrentPosition-4269);

        auto_functions.ArmUpDownTime(0.7, 400, 369);

        auto_functions.OpenServo();

        auto_functions.ArmUpDown(0.7, -369);

        auto_functions.DriveForward(0.9, 2069);

        ArmMotor_Right.getCurrentPosition();




    }
}