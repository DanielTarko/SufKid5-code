package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
/**
 * This is an autonomous program for FTC, Rover Ruckus, 2018-19
 * Created by Daniel Tarko on October 17, 2018
 */

@Autonomous(name="MarkerAuto", group="SufKid5")
public class MarkerAuto extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSufKid5         robot   = new HardwareSufKid5();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.8;
    static final double     SEARCH_SPEED = 0.15;
    static final double     SPIN_SPEED = 0.5;
    static final double     LIFT_SPEED    = 0.5;
    static final double     STRAFE_SPEED    = 0.3;
    static final double     PUSH_SPEED = 0.3;
    static final double     faceTwistTime    = 1.3;
    double     PUSH_TIME    = 1.2;
    double     FORWARD_TIME    = 0.3;
    double     twistTime    = 0;
    double     extraTwistTime    = 0;
    double     craterTwistTime    = 0;
    String direction = "none";



    private GoldAlignDetector detector;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        robot.marker.setPosition(0);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//Drop
        robot.liftL.setPower(-LIFT_SPEED);
        robot.liftR.setPower(-LIFT_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 6.4)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//move left
        robot.liftL.setPower(0);
        robot.liftR.setPower(0);

        robot.fr.setPower(-STRAFE_SPEED);
        robot.fl.setPower(STRAFE_SPEED);
        robot.br.setPower(STRAFE_SPEED);
        robot.bl.setPower(-STRAFE_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

//move forward a little so it doesnt get stuck on the hook


        robot.fr.setPower(STRAFE_SPEED);
        robot.fl.setPower(STRAFE_SPEED);
        robot.br.setPower(STRAFE_SPEED);
        robot.bl.setPower(STRAFE_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


//twist right to see right cube
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {

            robot.fr.setPower(SEARCH_SPEED);
            robot.fl.setPower(-SEARCH_SPEED);
            robot.br.setPower(SEARCH_SPEED);
            robot.bl.setPower(-SEARCH_SPEED);
        }
        robot.fr.setPower(0);
        robot.fl.setPower(0);
        robot.br.setPower(0);
        robot.bl.setPower(0);

//find cube

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.

        //wait a second
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//if cant be found twist left
        runtime.reset();
        while (opModeIsActive() && detector.getXPosition() == 0 && (runtime.seconds() < 3)) {
            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.

            robot.fr.setPower(-SEARCH_SPEED);
            robot.fl.setPower(SEARCH_SPEED);
            robot.br.setPower(-SEARCH_SPEED);
            robot.bl.setPower(SEARCH_SPEED);

            extraTwistTime = runtime.seconds() + 0.6;
        }


        //center
        runtime.reset();
        if(detector.getXPosition() < 275 && detector.getXPosition() > 0 && extraTwistTime == 0){
            direction = "center";
            FORWARD_TIME = 0.0;
            PUSH_TIME = PUSH_TIME - 0.3;
            craterTwistTime = craterTwistTime - 0.3;

            robot.fr.setPower(-SEARCH_SPEED);
            robot.fl.setPower(SEARCH_SPEED);
            robot.br.setPower(-SEARCH_SPEED);
            robot.bl.setPower(SEARCH_SPEED);
            while (opModeIsActive() && (runtime.seconds() < 1.25)) {

                telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.
            }
            robot.fr.setPower(0);
            robot.fl.setPower(0);
            robot.br.setPower(0);
            robot.bl.setPower(0);
        }

        //twist left
        while (opModeIsActive() && detector.getXPosition() < 465 && direction != "center"  && detector.getXPosition() != 0) {
            robot.fr.setPower(-SEARCH_SPEED);
            robot.fl.setPower(SEARCH_SPEED);
            robot.br.setPower(-SEARCH_SPEED);
            robot.bl.setPower(SEARCH_SPEED);

            twistTime = runtime.seconds() + extraTwistTime;
            direction = "left";
            craterTwistTime = twistTime;
            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.

        }

        //twist right
        if (opModeIsActive() && detector.getXPosition() > 300 && direction != "left" && direction != "center"  && detector.getXPosition() != 0) {


            twistTime = runtime.seconds();
            direction = "right";
            craterTwistTime = -twistTime;
            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4)) {

                robot.fr.setPower(SEARCH_SPEED);
                robot.fl.setPower(-SEARCH_SPEED);
                robot.br.setPower(SEARCH_SPEED);
                robot.bl.setPower(-SEARCH_SPEED);
            }

        }

        detector.disable();
//move forward to knock away cube
        robot.fr.setPower(FORWARD_SPEED);
        robot.fl.setPower(FORWARD_SPEED);
        robot.br.setPower(FORWARD_SPEED);
        robot.bl.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        if(direction == "left") {
            robot.fr.setPower(SEARCH_SPEED);
            robot.fl.setPower(-SEARCH_SPEED);
            robot.br.setPower(SEARCH_SPEED);
            robot.bl.setPower(-SEARCH_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < twistTime - 1)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        }

        if(direction == "right") {
            robot.fr.setPower(-SEARCH_SPEED);
            robot.fl.setPower(SEARCH_SPEED);
            robot.br.setPower(-SEARCH_SPEED);
            robot.bl.setPower(SEARCH_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < twistTime + 1.3)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        }

        robot.fr.setPower(FORWARD_SPEED);
        robot.fl.setPower(FORWARD_SPEED);
        robot.br.setPower(FORWARD_SPEED);
        robot.bl.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < FORWARD_TIME)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.fr.setPower(0);
        robot.fl.setPower(0);
        robot.br.setPower(0);
        robot.bl.setPower(0);
//drop marker and pause
        robot.marker.setPosition(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.marker.setPosition(0);
//push cube forward a little
        robot.fr.setPower(PUSH_SPEED);
        robot.fl.setPower(PUSH_SPEED);
        robot.br.setPower(PUSH_SPEED);
        robot.bl.setPower(PUSH_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < PUSH_TIME)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//back up
        robot.fr.setPower(-PUSH_SPEED);
        robot.fl.setPower(-PUSH_SPEED);
        robot.br.setPower(-PUSH_SPEED);
        robot.bl.setPower(-PUSH_SPEED);

        if (direction == "right"){
            PUSH_TIME = PUSH_TIME - 0.8;
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < PUSH_TIME)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

//go up again
        /*
        robot.liftL.setPower(LIFT_SPEED);
        robot.liftR.setPower(LIFT_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 6)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.liftL.setPower(0);
        robot.liftR.setPower(0);
        */

    //turn left to face crater
        robot.fr.setPower(-SPIN_SPEED);
        robot.fl.setPower(SPIN_SPEED);
        robot.br.setPower(-SPIN_SPEED);
        robot.bl.setPower(SPIN_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < faceTwistTime + craterTwistTime/6)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
/*
    //drive forward a little
        if (direction == "right")
        robot.fr.setPower(FORWARD_SPEED);
        robot.fl.setPower(FORWARD_SPEED);
        robot.br.setPower(FORWARD_SPEED);
        robot.bl.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        */
//wait for 0.25 seconds
        robot.fr.setPower(0);
        robot.fl.setPower(0);
        robot.br.setPower(0);
        robot.bl.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.25)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//if right reposition in front of marker and cube
        if(direction == "right") {
            //strafe a little
            robot.fr.setPower(-STRAFE_SPEED);
            robot.fl.setPower(STRAFE_SPEED);
            robot.br.setPower(STRAFE_SPEED);
            robot.bl.setPower(-STRAFE_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            //forward a little
            robot.fr.setPower(STRAFE_SPEED);
            robot.fl.setPower(STRAFE_SPEED);
            robot.br.setPower(STRAFE_SPEED);
            robot.bl.setPower(STRAFE_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.75)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        }

    //strafe left into wall to line up robot
        robot.fr.setPower(-STRAFE_SPEED);
        robot.fl.setPower(STRAFE_SPEED);
        robot.br.setPower(STRAFE_SPEED);
        robot.bl.setPower(-STRAFE_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Full speed ahead into the crater!!! YAY!
        robot.fr.setPower(FORWARD_SPEED);
        robot.fl.setPower(FORWARD_SPEED);
        robot.br.setPower(FORWARD_SPEED);
        robot.bl.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4.75)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.fr.setPower(0);
        robot.fl.setPower(0);
        robot.br.setPower(0);
        robot.bl.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
