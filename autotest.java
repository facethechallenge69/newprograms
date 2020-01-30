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

@TeleOp(name = "AutoBest", group = "Tutorials")
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
    private Servo servoarm;
    private Servo shake_shack_servo;
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    Orientation angles;

    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;

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
        servoarm = hardwareMap.servo.get("servo_arm");

        shake_shack_servo = hardwareMap.servo.get("servo_arm");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int encoder_tics = 0;
        double encoder_speed = 0;
        double last_encode_speed = 7;
        int last_encoder_tics = 7;

        double servo_arm_position = 0;
        double last_servo_arm_postition = 7;

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
            if (gamepad1.dpad_down)
            {
                //drive back
                auto_functions.DriveForward(encoder_speed, -encoder_tics);
            }

            if (gamepad2.dpad_left)
            {
                auto_functions.StrafeLeft(encoder_speed,encoder_tics);
            }
            if (gamepad2.dpad_right)
            {
                auto_functions.StrafeRight(encoder_speed,encoder_tics);
            }
            if (gamepad2.dpad_up)
            {
                auto_functions.ArmUpDown(encoder_speed, encoder_tics);
            }

            if (gamepad1.right_bumper) {
                auto_functions.CloseServo();
            }

            if (gamepad1.left_bumper) {
                auto_functions.OpenServo();
            }

            if (gamepad2.x)
            {
                servo_arm_position += 0.05;
                sleep(100);
            }

            if(gamepad2.a)
            {
                servo_arm_position -= 0.01;
                sleep(100);
            }

            if (gamepad2.right_bumper)
            {
                shake_shack_servo.setPosition(servo_arm_position);
            }



            if((last_encoder_tics != encoder_tics) || (last_encode_speed !=  encoder_speed) || (last_servo_arm_postition != servo_arm_position))
            {
                last_encoder_tics = encoder_tics;
                last_encode_speed = encoder_speed;



















                last_servo_arm_postition = servo_arm_position;
                telemetry.addData("encoder distance %d", encoder_tics);
                telemetry.addData("encoder speed %f", encoder_speed);
                telemetry.addData("servoposition %d", servo_arm_position);
                telemetry.addData("ArmMotorRightPosition %d", ArmMotor_Right.getCurrentPosition());

                telemetry.addLine("gampad1.a = + 50 encoder tics");
                telemetry.addLine("gampad1.x = - 50 encoder tics");
                telemetry.addLine("gampad1.b = + 0.05 encoder speed");
                telemetry.addLine("gampad1.y = - 0.05 encoder speed");

                telemetry.addLine("gampad1.dpad_up = drive forward");
                telemetry.addLine("gampad1.dpad_left = turn left");
                telemetry.addLine("gampad1.dpad_right = turn right");
                telemetry.addLine("gampad1.dpad_down = drive back");

                telemetry.addLine("gampad2.dpad_right = straferight");
                telemetry.addLine("gampad2.dpad_left = strafeleft");
                telemetry.addLine("gampad2.dpad_up = armUpDown");

                telemetry.addLine("gampad1.right_bumper = close servo");
                telemetry.addLine("gampad1.left_bumper = open servo");

                telemetry.addLine("gamepad2.right_bumper = shakeservo doub");

                telemetry.addLine("gamepad2.x = +0.05");
                telemetry.addLine("gamepad2.a = -0.05");

                telemetry.update();
            }
            




            idle();
        }
    }

}





