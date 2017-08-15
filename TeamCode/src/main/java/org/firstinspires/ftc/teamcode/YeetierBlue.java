/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="YEETIER", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class YeetierBlue extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor intake = null;
    private DcMotor shooter = null;

    Servo bacon;
    ColorSensor colorSensor;


    int baconDist = 200;



    double up =  .5;
    double down  = .5;
    int l1=500;
    int s1 = 1000;
    int r1 = 500;
    int s2 = 500;
    int r3 = 500;
    int s3 = 500;
    int s4 = 200;
    int s5 = 500;
    int r4 = 500;
    int s6 = 500;
    int r5 = 500;
    int s7 = 500;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");


        leftMotor  = hardwareMap.dcMotor.get("left motor");
        intake = hardwareMap.dcMotor.get("in");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        shooter  = hardwareMap.dcMotor.get("shooter");
        bacon = hardwareMap.servo.get("bacon");
        colorSensor = hardwareMap.colorSensor.get("color");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        shooter.setPower(-1);
        sleep(500);
        intake.setPower(-1);
        sleep(2000);
        shooter.setPower(0);
        intake.setPower(0);
        sleep(1000);
        move(0,.75,l1);
        move(.75,.75, s1);
        move(.75,0, r1);
        move(.75,.75,s2);
        move(.75,0,r3);
        move(.75,.75,s3);
        double redDist = Math.sqrt(Math.pow(1.0-colorSensor.red(),2) + Math.pow(colorSensor.green(),2)+Math.pow(colorSensor.blue(),2));
        double blueDist = Math.sqrt(Math.pow(colorSensor.red(),2) + Math.pow(colorSensor.green(),2)+Math.pow(1-colorSensor.blue(),2));
        boolean isBlue = blueDist<blueDist;


        if(isBlue){
            bacon.setPosition(0);
            wait(baconDist);
            bacon.setPosition(1);
            wait(baconDist);
            bacon.setPosition(.5);
            move(.75,.75,s4+s5);
        }
        else{
            move(.75,.75,s4);
            bacon.setPosition(0);
            wait(baconDist);
            bacon.setPosition(1);
            wait(baconDist);
            bacon.setPosition(.5);
            move(.75,.75,s5);
        }
        move(.75,.0,r4);
        move(.75,.75,s6);

        move(.75,0.0,r5);
        move(.75,.75,s7);


        runtime.reset();


    }

    void move(double l, double r, int d) throws  InterruptedException{
        leftMotor.setPower(l);
        rightMotor.setPower(r);
        sleep(d);
        leftMotor.setPower(0);
        rightMotor.setPower(0);



    }
}
