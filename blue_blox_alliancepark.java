/*main PACKAGE.
package org.firstinspires.ftc.teamcode.newprograms;

//hardware imports
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//movement of the motors and servos and color sensor imports
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//random imports
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.newprograms.autofunctions;

import static android.os.SystemClock.sleep;

@Autonomous(name = "blueblock alliance park", group = "Tutorials")
public class blue_blox_alliancepark extends LinearOpMode
{
    //all the wheel motors
    private DcMotor motorL_Up;
    private DcMotor motorL_Down;
    private DcMotor motorR_Up;
    private DcMotor motorR_Down;

    //more servos
    private Servo RedServo;
    private Servo BlackServo;

    //armmotor
    private DcMotor ArmMotor_Left;
    private DcMotor ArmMotor_Right;

    //servo
    private Servo armservo;
    private Servo shake_shack_servo;

    //Sleep calling
    private ElapsedTime runtime = new ElapsedTime();

    private Servo side_servo;
    private Servo side_servo_claw;


    private DcMotor Side_Arm_Gang;


    //gyro
    BNO055IMU imu;

    //gyro stuff
    Orientation angles;

    //calling colorsensors
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    NormalizedColorSensor colorSensor3;
    NormalizedColorSensor colorSensor4;

    //calling auto_functions.
    autofunctions auto_functions = new autofunctions();

    //Setting the Current Position integer to 0
    int CurrentPosition = 0;

    int ArmPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Receiving the necessary hardware for the motors
        motorL_Down = hardwareMap.dcMotor.get("left_motor_d");
        motorR_Down = hardwareMap.dcMotor.get("right_motor_d");
        motorL_Up = hardwareMap.dcMotor.get("left_motor_up");
        motorR_Up = hardwareMap.dcMotor.get("right_motor_up");

        //hardware for servos
        RedServo = hardwareMap.servo.get("red_servo");
        BlackServo = hardwareMap.servo.get("black_servo");

        //more motors!
        ArmMotor_Left = hardwareMap.dcMotor.get("armmotor_l");
        ArmMotor_Right = hardwareMap.dcMotor.get("armmotor_r");

        //one more servo
        armservo = hardwareMap.servo.get("arm_servo");
        shake_shack_servo = hardwareMap.servo.get("servo_arm");



        //colors
        colorSensor1 = (NormalizedColorSensor) hardwareMap.colorSensor.get("red_color");
        colorSensor2 = (NormalizedColorSensor) hardwareMap.colorSensor.get("black_color");
        colorSensor3 = (NormalizedColorSensor) hardwareMap.colorSensor.get("red_color.2");
        colorSensor4 =   (NormalizedColorSensor) hardwareMap.colorSensor.get("black_color.2");
        side_servo = hardwareMap.servo.get("side_servo");
        side_servo_claw = hardwareMap.servo.get("side_servo_gang");

       // Side_Arm_Gang = hardwareMap.dcMotor.get("gang_side_arm");


        //potential gyro, we will just let it stay here
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Setting the behavior for the motors to brake.
        motorR_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmMotor_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Side_Arm_Gang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initializing from autofunctions.java

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
                colorSensor3,
                colorSensor4,
                side_servo,
                side_servo_claw,

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


        auto_functions.bluud_3_comp(-250);


        auto_functions.DriveForward(0.6,-1469);


        auto_functions.TurnRight(0.369, 800);

        sleep(100);

        auto_functions.DriveForward(0.269,-300);

        auto_functions.OpenServo();

        sleep(100);

        auto_functions.DriveForward(0.269,300);

        sleep(100);

        auto_functions.TurnLeft(0.369, 839);

        sleep(100);

        auto_functions.CloseServo();

        auto_functions.DriveForward(0.769, 1469);


    }
}
*/
