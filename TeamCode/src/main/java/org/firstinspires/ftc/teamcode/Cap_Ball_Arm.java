package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Pranav on 2/2/2017.
 */
@TeleOp(name="Cap Ball Arm Test", group="Pushbot")
public class Cap_Ball_Arm extends OpMode{

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


    if (gamepad2.right_trigger>.5)
        robot.capBallMotorA.setPower(.5);

    if (gamepad2.left_trigger>.5)
        robot.capBallMotorB.setPower(.5);
}
    {
    }
};