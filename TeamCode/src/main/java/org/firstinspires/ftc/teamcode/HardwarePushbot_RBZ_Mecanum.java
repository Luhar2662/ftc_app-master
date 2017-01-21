package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot_RBZ_Mecanum
{
    /* Public OpMode members. */
    public DcMotor  leftMotorF   = null;
    public DcMotor  rightMotorF  = null;
    public DcMotor  leftMotorB   = null;
    public DcMotor  rightMotorB  = null;
    public Servo    beaconRight  =null;
    public Servo    beaconLeft  =null;

    public ColorSensor  ColorSensor  =null;

    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot_RBZ_Mecanum(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotorF   = hwMap.dcMotor.get("left_F_drive");
        rightMotorF  = hwMap.dcMotor.get("right_F_drive");
        leftMotorB   = hwMap.dcMotor.get("left_B_drive");
        rightMotorB  = hwMap.dcMotor.get("right_B_drive");
        beaconRight  = hwMap.servo.get("Right_Beacon");
        beaconLeft  =   hwMap.servo.get("Left_Beacon");
       ColorSensor = hwMap.colorSensor.get("sensor_color");

//        armMotor    = hwMap.dcMotor.get("left_arm");
        leftMotorF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotorB.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorF.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotorB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
  //      armMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
  //      leftClaw = hwMap.servo.get("left_hand");
  //      rightClaw = hwMap.servo.get("right_hand");
        beaconLeft.setPosition(Servo.MAX_POSITION);
        beaconLeft.setPosition(Servo.MAX_POSITION);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

