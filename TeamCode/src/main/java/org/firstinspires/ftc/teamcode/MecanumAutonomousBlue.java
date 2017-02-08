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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;



@Autonomous(name="Mecanums Blue Side", group="Pushbot")
//@disabled
public class MecanumAutonomousBlue extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot_RBZ_Mecanum         robot   = new HardwarePushbot_RBZ_Mecanum();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        // Wait for the game to start (driver presses PLAY)
        waitForStart();
/*
        sleep(6000);

        robot.beaconRight.setPosition(0);
        robot.rightMotorF.setPower(.5);
        robot.rightMotorB.setPower(.5);
        robot.leftMotorF.setPower(.5);
        robot.leftMotorB.setPower(.5);
        robot.beaconRight.setPosition(.5);

        sleep(4000);

        robot.rightMotorF.setPower(-.3);
        robot.rightMotorB.setPower(-.3);
        robot.leftMotorF.setPower(-.3);
        robot.leftMotorB.setPower(-.3);

       sleep(1500);
        /*robot.beaconRight.setPosition(1);
        robot.beaconLeft.setPosition(1);
*/
        robot.rightMotorF.setPower(-.2);
        robot.rightMotorB.setPower(.2);
        robot.leftMotorF.setPower(.2);
        robot.leftMotorB.setPower(-.2);
        sleep(4000);


        robot.rightMotorF.setPower(.2);
        robot.rightMotorB.setPower(.2);
        robot.leftMotorF.setPower(.2);
        robot.leftMotorB.setPower(.2);
        sleep(2000);

        while (robot.ColorSensor.red()<1 && robot.ColorSensor.blue()<1){
            robot.rightMotorF.setPower(-.8);
            robot.rightMotorB.setPower(.8);
            robot.leftMotorF.setPower(.8);
            robot.leftMotorB.setPower(-.8);
        }
        while (robot.ColorSensor.blue() > robot.ColorSensor.red()){
            //sees blue
            robot.beaconRight.setPosition(.5);


            robot.rightMotorF.setPower(.2);
            robot.rightMotorB.setPower(.2);
            robot.leftMotorF.setPower(.2);
            robot.leftMotorB.setPower(.2);

            sleep(2000);
            //un-initialize

            robot.rightMotorF.setPower(-.2);
            robot.rightMotorB.setPower(-.2);
            robot.leftMotorF.setPower(-.2);
            robot.leftMotorB.setPower(-.2);
            robot.beaconRight.setPosition(1);
            sleep(2000);

            robot.leftMotorF.setPower(0);
            robot.leftMotorB.setPower(0);
            robot.rightMotorF.setPower(0);
            robot.rightMotorB.setPower(0);
        }
        while (robot.ColorSensor.red() > robot.ColorSensor.blue()) {
            //sees red
            while (robot.ColorSensor.blue() < robot.ColorSensor.red()){
                robot.rightMotorF.setPower(-.8);
                robot.rightMotorB.setPower(.8);
                robot.leftMotorF.setPower(.8);
                robot.leftMotorB.setPower(-.8);
            }
            robot.beaconLeft.setPosition(.5);
            robot.rightMotorF.setPower(.4);
            robot.rightMotorB.setPower(.4);
            robot.leftMotorF.setPower(.4);
            robot.leftMotorB.setPower(.4);

            sleep(2000);
            //un-initialize
            robot.beaconLeft.setPosition(1);
            robot.rightMotorF.setPower(-.4);
            robot.rightMotorB.setPower(-.4);
            robot.leftMotorF.setPower(-.4);
            robot.leftMotorB.setPower(-.4);
        }


stop();



            //  sleep(250);   // optional pause after each move
        }

}
