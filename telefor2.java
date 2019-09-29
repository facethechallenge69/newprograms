package org.firstinspires.ftc.teamcode.newprograms;
//needs to be fixed
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "telefor2", group = "Tutorials")
public class

telefor2 extends LinearOpMode {
    private DcMotor motorL_Down;
    private DcMotor motorR_Down;
    private DcMotor motorR_Up;
    private DcMotor motorL_Up;
    


    private Servo RedServo;
    private Servo BlackServo;


    int MOV_LEFT_RIGHT = 1;
    int MOV_FRONT_BACK = 2;
    int STRAF_LEFT = 3;
    int STAF_RIGHT = 4;

    //private Telemetry telemetry;


    @Override
    public void runOpMode() throws InterruptedException {


        motorL_Down = hardwareMap.dcMotor.get("left_motor_d");
        motorR_Down = hardwareMap.dcMotor.get("right_motor_d");
        motorL_Up = hardwareMap.dcMotor.get("left_motor_up");
        motorR_Up = hardwareMap.dcMotor.get("right_motor_up");
        

        RedServo = hardwareMap.servo.get("red_servo");
        BlackServo = hardwareMap.servo.get("black_servo");


        double motorSpeed = 1;

        
        int moving = 0;

        double red_value = 0;

        double black_value = 1;






        motorR_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL_Down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL_Up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);




        BlackServo.setPosition(1);
        RedServo.setPosition(1);
        waitForStart();

        while (opModeIsActive()) {
            //moves wheels forward and backward

            if (gamepad1.a) {
                motorSpeed = 1;
            }

            if (gamepad1.b) {
                motorSpeed = 0.4;
            }



            //incremental movement for RedServo




            //moves mecanum wheels forward and backward

            if(moving == 0 && gamepad1.left_stick_y !=0 ) {

                moving = MOV_FRONT_BACK;
                motorL_Down.setPower(motorSpeed * -gamepad1.left_stick_y);
                motorR_Down.setPower(motorSpeed * gamepad1.left_stick_y);
                motorL_Up.setPower(motorSpeed * gamepad1.left_stick_y);
                motorR_Up.setPower(motorSpeed * -gamepad1.left_stick_y);
            }
            else if(gamepad1.left_stick_y ==0 && moving == MOV_FRONT_BACK)
            {
                moving = 0;
            }


            //turns mecanum wheels left and right

            if(moving == 0 && gamepad1.left_stick_x !=0 ) {

                moving = MOV_LEFT_RIGHT;
                motorL_Down.setPower(motorSpeed * -gamepad1.left_stick_x);
                motorR_Down.setPower(motorSpeed * -gamepad1.left_stick_x);
                motorL_Up.setPower(motorSpeed * gamepad1.left_stick_x);
                motorR_Up.setPower(motorSpeed * gamepad1.left_stick_x);
            }
            else if(gamepad1.left_stick_x ==0 && moving == MOV_LEFT_RIGHT)
            {
                moving = 0;
            }





        /*    if (moving == 0 && gamepad1.dpad_right) {
                moving = STAF_RIGHT;
                motorL_Down.setPower(1);
                motorR_Down.setPower(1);
                motorL_Up.setPower(-1);
                motorR_Up.setPower(-1);
            }
            else if(!gamepad1.dpad_right && moving == STAF_RIGHT)
            {
                moving = 0;
            }



            if (moving == 0 && gamepad1.dpad_left) {
                moving = STRAF_LEFT;
                motorL_Down.setPower(-1);
                motorR_Down.setPower(-1);
                motorL_Up.setPower(1);
                motorR_Up.setPower(1);
            }

            else if(!gamepad1.dpad_left && moving == STRAF_LEFT)
            {
                moving = 0;
            }

            if(moving == 0)
            {
                motorL_Down.setPower(0);
                motorR_Down.setPower(0);
                motorL_Up.setPower(0);
                motorR_Up.setPower(0);
            }

*/

         

            //___________________________________________________
            // red/black servo movement

//dpad up/down is for red serv
//dppad right/left is for black servo
//up dpad + 0.1
// down dpad -0.1
// left dpad +0.1
// right dpad -0.1

            if(gamepad1.dpad_up)
            {
                red_value = red_value + 0.1;
                RedServo.setPosition(red_value);
            }

            if(gamepad1.dpad_down)
            {
                red_value = red_value - 0.1;
                RedServo.setPosition(red_value);
            }

            if (gamepad1.dpad_left) {
                black_value = black_value + 0.1;
                BlackServo.setPosition(black_value);
            }

            if (gamepad1.dpad_right)
            {
                black_value = black_value - 0.1;
                BlackServo.setPosition(black_value);
            }






            //_______________________motor claw movement_______________

           

           
            telemetry.update();
            idle();

        }


    }

    //void for claw
  /*  public void ClawForward(double Power, int Distance) {


        //Reset Encoders
        motorArmClaw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Set to RUN_TO_POSITION Mode
        motorArmClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //Set Target Position
        motorArmClaw.setTargetPosition(Distance);


        //Setting Motor Power
        motorArmClaw.setPower(Power);


        //While Loop to Make Sure Encoders do Not Deactivate
        while (motorArmClaw.isBusy()) {
            //Wait until the task is done
        }


    }
   */
    //void for Circ


}


