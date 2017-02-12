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
TORT (INCLUDING NEGLIGEN
CE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RBZ Mecanum TeleOp By Joystick", group="Pushbot")
//@Disabled
public class PushbotTeleopTank_Iterative_RBZ_Mecanum_rev_Left_stick_x extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot_RBZ_Mecanum robot       = new HardwarePushbot_RBZ_Mecanum(); // use the class created to define a Pushbot's hardware

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

     /* Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

public void loop() {

    robot.rightMotorF.setPower(gamepad1.right_stick_x*.5);
    robot.rightMotorB.setPower(gamepad1.right_stick_x*.5);
    robot.leftMotorF.setPower(-gamepad1.right_stick_x*.5);
    robot.leftMotorB.setPower(-gamepad1.right_stick_x*.5);

    if(gamepad1.left_stick_x>.25)
    {   robot.rightMotorF.setPower(-gamepad1.left_stick_x);
        robot.rightMotorB.setPower(gamepad1.left_stick_x);
        robot.leftMotorF.setPower(gamepad1.left_stick_x);
        robot.leftMotorB.setPower(-gamepad1.left_stick_x);}

    if(gamepad1.left_stick_x<-.25)
    { robot.rightMotorF.setPower(-gamepad1.left_stick_x);
        robot.rightMotorB.setPower(gamepad1.left_stick_x);
        robot.leftMotorF.setPower(gamepad1.left_stick_x);
        robot.leftMotorB.setPower(-gamepad1.left_stick_x);}

    if(gamepad1.left_stick_y>.25)
    { robot.rightMotorF.setPower(-gamepad1.left_stick_y);
        robot.rightMotorB.setPower(-gamepad1.left_stick_y);
        robot.leftMotorF.setPower(-gamepad1.left_stick_y);
        robot.leftMotorB.setPower(-gamepad1.left_stick_y);}

    if(gamepad1.left_stick_y<-.25)
    {    robot.rightMotorF.setPower(-gamepad1.left_stick_y);
        robot.rightMotorB.setPower(-gamepad1.left_stick_y);
        robot.leftMotorF.setPower(-gamepad1.left_stick_y);
        robot.leftMotorB.setPower(-gamepad1.left_stick_y);}



    if(gamepad1.right_bumper){
        robot.beaconRight.setPosition(.5);
         }

    if(gamepad1.left_bumper) {
        robot.beaconLeft.setPosition(-.5);
    }

    if(gamepad1.right_trigger>.5){
        robot.beaconRight.setPosition(0);
    }

    if(gamepad1.left_trigger>.5){
        robot.beaconLeft.setPosition(1);}
    if(gamepad2.left_bumper==true){
      robot.ArmServoLeft.setPower(10);
      robot.ArmServoRight.setPower(-10);}
    if (gamepad2.right_bumper==true) {
        robot.ArmServoLeft.setPower(-.5);
        robot.ArmServoRight.setPower(.5);}
    robot.capBallMotorB.setPower(gamepad2.right_stick_y);
    robot.capBallMotorA.setPower(-gamepad2.right_stick_y);

}
    public void stop () {
    }
    }