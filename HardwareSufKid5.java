
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is an hardware map of SufKid5
 * for FTC, Rover Ruckus, 2018-19
 * Created by Daniel Tarko on October 17, 2018 */
public class HardwareSufKid5
{
    /* Public OpMode members. */
    public DcMotor  fr   = null;//creates dc motor and gives names also sets motor value to 0
    public DcMotor  fl  = null;
    public DcMotor  bl  = null;
    public DcMotor  br   = null;//creates dc motor and gives names also sets motor value to 0
    public DcMotor  liftL  = null;
    public DcMotor  liftR  = null;

    public Servo    marker = null;


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();//private bc it is only used in this file

    /* Constructor */
    public HardwareSufKid5() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        fr   = hwMap.dcMotor.get("fr");//names the dc motot and on the phone we say which motor is attached to which core mottor controler
        fl  = hwMap.dcMotor.get("fl");
        br  = hwMap.dcMotor.get("br");
        bl   = hwMap.dcMotor.get("bl");//names the dc motot and on the phone we say which motor is attached to which core mottor controler
        liftL  = hwMap.dcMotor.get("liftL");
        liftR  = hwMap.dcMotor.get("liftR");
        liftR.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);


        marker = hwMap.servo.get("marker");

        // Set all motors to zero power
        fl.setPower(0);//so that robot doesnt move when code is run
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        liftL.setPower(0);
        liftR.setPower(0);

        marker.setPosition(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

