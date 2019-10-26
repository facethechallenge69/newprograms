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
    private DcMotor motor_UpDown;
    private DcMotor motor_SideSide;

    private Servo RedServo;
    private Servo BlackServo;
    private Servo ArmServo;

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
        motor_UpDown = hardwareMap.dcMotor.get("motor_up.up");
        motor_SideSide = hardwareMap.dcMotor.get("motor_side.side");


        RedServo = hardwareMap.servo.get("red_servo");
        BlackServo = hardwareMap.servo.get("black_servo");

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

        motor_UpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_SideSide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        auto_functions.Initialize(motorL_Down,
                motorR_Down,
                motorR_Up,
                motorL_Up,
                motor_UpDown,
                motor_SideSide,
                RedServo,
                BlackServo,
                ArmServo,
                imu,
                telemetry);

        //Waiting for the user to press start
        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.a)
            {
                encoder_tics += 100;
                sleep(400);
            }
            if (gamepad1.b)
            {
                encoder_speed += 0.1;
                sleep(400);
            }
            if(gamepad1.y)
            {
                encoder_speed -= 0.1;
                sleep(400);
            }
            if (gamepad1.x)
            {
                encoder_tics -= 50;
                sleep(400);
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


            telemetry.addData("encoder distance %d", encoder_tics);
            telemetry.addData("encoder speed %f", encoder_speed);
            telemetry.update();

            idle();
        }
    }

}





