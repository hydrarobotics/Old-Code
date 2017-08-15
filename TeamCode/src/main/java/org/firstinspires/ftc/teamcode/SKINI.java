package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Hydra Robotics on 11/4/2016.
 */
@TeleOp(name="FLAP TeleOP", group="Iterative Opmode")
public class SKINI extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor intake = null;
    private DcMotor shotsFiredPewPew = null;


    double forw =  1;
    double back  = 0;
    double stop = .5;

    boolean isShooting = false;
    /*
 * Code to run ONCE when the driver hits INIT
 */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        intake = hardwareMap.dcMotor.get("in");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        shotsFiredPewPew = hardwareMap.dcMotor.get("shooter");
        shotsFiredPewPew.setDirection(DcMotorSimple.Direction.REVERSE);


        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        leftMotor.setPower(-gamepad1.left_stick_y);
        rightMotor.setPower(-gamepad1.right_stick_y);
        intake.setPower(-gamepad2.right_stick_y);

        if(isShooting){
            shotsFiredPewPew.setPower(1);
            if(gamepad2.a)
                isShooting = false;
        }
        else{
            shotsFiredPewPew.setPower(0);
        }
        if(gamepad2.b)
            isShooting = true;


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
