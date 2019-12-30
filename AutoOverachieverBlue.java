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

@Autonomous(name = "BoverachieverlueBlocks.", group = "Tutorials")
public class AutoOverachieverBlue extends LinearOpMode
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

    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    Orientation angles;

    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;

    autofunctions auto_functions = new autofunctions();

    int CurrentPosition = 0;


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

        ArmMotor_Left = hardwareMap.dcMotor.get("armmotor_l");
        ArmMotor_Right = hardwareMap.dcMotor.get("armmotor_r");

        armservo = hardwareMap.servo.get("arm_servo");
        shake_shack_servo = hardwareMap.servo.get("servo_arm");

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

        auto_functions.ServoUp();

        ArmMotor_Right.getCurrentPosition();

        waitForStart();

        ArmMotor_Right.getCurrentPosition();

        auto_functions.DriveForward(0.5, -1600);

        sleep(250);

        auto_functions.StrafeLeft(0.5,225);

        CurrentPosition = auto_functions.StrafeLeftColor(0.2, 2750);
        if(CurrentPosition < 0)
            CurrentPosition = CurrentPosition * -1;

        sleep(250);

        auto_functions.getcube();

        sleep(100);

        auto_functions.DriveForward(0.5, 200);

        sleep(100);

        auto_functions.TurnLeft(0.5,1250);

        sleep(100);

        auto_functions.ArmUpDown(0.5, 369);

        sleep(100);

        telemetry.addData("CurrentPosition", "%d", CurrentPosition);
        auto_functions.DriveForward(0.5, -1500-CurrentPosition);

        auto_functions.ArmUpDown(0.5, -269);

        auto_functions.DriveForward(0.5,-2000);

        sleep(100);

        //Drive Forward a bit more at top

        //Have a side arm or something that can turn the foundation liek so that the long side is against the wall

        auto_functions.ArmUpDown(0.5, 500);

        auto_functions.OpenServo();

        auto_functions.DriveForward(0.35, 2250);

        //Go back to position of sensing, keeping arm up but not blocking color sensor

        //Sense color from Current Position thing confusing pls stop help

        //Make autofunctions that gets second cube and just brings the arm down at the spot of the black cube.

        //Huff and Puff to the Placing of the second cube basically just repeats the thing above.

        //Go back and park.


    }
}