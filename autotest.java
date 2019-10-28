package org.firstinspires.ftc.teamcode.newprograms;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

@TeleOp(name = "1/26 Autonomous Test Program", group = "Tutorials")
public class autotest extends LinearOpMode
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

    Telemetry telemetry;

    autofunctions auto_functions = new autofunctions();


    //  Telemetry telemetry = new Telemetry();


    @Override
    public void runOpMode() throws InterruptedException
    {
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
                ArmMotor_Left,
                ArmMotor_Right,
                RedServo,
                BlackServo,
                armservo,
                imu,
                telemetry);

        //Waiting for the user to press start
        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.a)
            {
                encoder_tics += 50;
                sleep(200);
            }
            if (gamepad1.b)
            {
                encoder_speed += 0.05;
                sleep(200);
            }
            if(gamepad1.y)
            {
                encoder_speed -= 0.05;
                sleep(200);
            }
            if (gamepad1.x)
            {
                encoder_tics -= 25;
                sleep(200);
            }
            if (gamepad1.dpad_up)
            {
                auto_functions.DriveForward(encoder_speed, encoder_tics);
            }
            if (gamepad1.dpad_left)
            {
                auto_functions.TurnLeft(encoder_speed, encoder_tics);
            }
            if (gamepad1.dpad_right)
            {
                auto_functions.TurnRight(encoder_speed, encoder_tics);
            }
            if (gamepad2.dpad_left)
            {
                auto_functions.StrafeLeft(encoder_speed,encoder_tics);
            }
            if (gamepad2.dpad_right)
            {
                auto_functions.StrafeRight(encoder_speed,encoder_tics);
            }


            telemetry.addData("encoder distance %d", encoder_tics);
            telemetry.addData("encoder speed %f", encoder_speed);
            telemetry.update();

            idle();
        }
    }

}





