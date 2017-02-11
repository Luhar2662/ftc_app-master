package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwarePushbot_RBZ_Mecanum
{
    /* Public OpMode members. */
    public DcMotor  leftMotorF   = null;
    public DcMotor  rightMotorF  = null;
    public DcMotor  leftMotorB   = null;
    public DcMotor  rightMotorB  = null;
    public Servo    beaconRight  =null;
    public Servo    beaconLeft  =null;
    public CRServo armMotorLeft  =null;
    public CRServo    armMotorRight  =null;
    public DcMotor  capBallMotorA =null;
    public DcMotor  capBallMotorB =null;

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
        armMotorLeft  = hwMap.crservo.get("ArmServoLeft");
        armMotorRight  =   hwMap.crservo.get("ArmServoRight");
       ColorSensor = hwMap.colorSensor.get("sensor_color");
        capBallMotorA= hwMap.dcMotor.get("armMotor");
        capBallMotorB= hwMap.dcMotor.get("armMotor2");

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


        leftMotorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



       beaconLeft.setPosition(Servo.MAX_POSITION);
        beaconRight.setPosition(0);
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

