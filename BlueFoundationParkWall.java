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

@Autonomous(name = "BlueFoundationWallPark", group = "Tutorials")
public class BlueFoundationParkWall extends LinearOpMode
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

    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    Orientation angles;

    NormalizedColorSensor colorSensor;
    NormalizedColorSensor colorSensor2;

    autofunctions auto_functions = new autofunctions();


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
                imu,

                telemetry);

        auto_functions.ServoUp();

        waitForStart();

        auto_functions.DriveForward(0.45, 1000);
        sleep(100);

        auto_functions.DriveForward(0.25,900);
        sleep(1000);

        auto_functions.ServoDown();
        sleep(1000);

        auto_functions.DriveForward(0.45, -1800);
        sleep(100);

        auto_functions.ServoUp();
        sleep(100);

        auto_functions.StrafeRight(0.45, 2050);
        sleep(100);

        auto_functions.DriveForward(0.45, 2400);
        sleep(100);

        auto_functions.StrafeLeft(0.45,2250);
        sleep(100);

        auto_functions.DriveForward(0.45, -1300);
        sleep(7000);

        auto_functions.StrafeRight(0.45, 2500);
        sleep(100);

        auto_functions.DriveForward(0.45,-1000);
        sleep(100);

        auto_functions.StrafeRight(0.45,1000);
    }
}
