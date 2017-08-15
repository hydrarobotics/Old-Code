package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Hydra Robotics on 11/4/2016.
 */
@TeleOp(name="THICC TeleOP", group="Iterative Opmode")
public class AyyLmao extends OpMode {

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

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


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
        if(gamepad1.left_bumper) {
            rightMotor.setPower(gamepad1.left_stick_y*(gamepad1.right_bumper?.5:1));
            leftMotor.setPower(gamepad1.right_stick_y*(gamepad1.right_bumper?.5:1));
        }else {
            leftMotor.setPower(-gamepad1.left_stick_y*(gamepad1.right_bumper?.5:1));
            rightMotor.setPower(-gamepad1.right_stick_y*(gamepad1.right_bumper?.5:1));
        }

        intake.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

        if(isShooting){
            shotsFiredPewPew.setPower(-1.0);
            if(gamepad1.a)
                isShooting = false;
        }else{
            if(gamepad1.y)
                shotsFiredPewPew.setPower(0.5);
            else
                shotsFiredPewPew.setPower(0.0);
        }
        if(gamepad1.b)
            isShooting = true;









    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
