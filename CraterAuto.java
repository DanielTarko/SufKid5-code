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

@Autonomous(name="CraterAuto", group="SufKid5")
public class CraterAuto extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSufKid5         robot   = new HardwareSufKid5();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.8;
    static final double     SEARCH_SPEED = 0.15;;
    static final double     STRAFE_SPEED    = 0.3;
    static final double     LIFT_SPEED = 0.5;

    double     CUBE_PUSH_TIME    = 0.9;
    double     FORWARD_MOVE_TIME    = 0;
    double     TURN_LEFT_TIME    = 0;
    double     FORWARD_TIME    = 0.3;
    double     twistTime    = 0;
    double     extraTwistTime    = 0;
    double     craterTwistTime    = 0;
    static final double     BACKUP_DISTANCE = 0.4;
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
        robot.marker.setPosition(1);


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
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
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
        while (opModeIsActive() && detector.getXPosition() == 0 && (runtime.seconds() < 2.9)) {
            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.

            robot.fr.setPower(-SEARCH_SPEED);
            robot.fl.setPower(SEARCH_SPEED);
            robot.br.setPower(-SEARCH_SPEED);
            robot.bl.setPower(SEARCH_SPEED);

            extraTwistTime = runtime.seconds() + 0.6;
        }


        //center
        runtime.reset();
        if(detector.getXPosition() < 375 && detector.getXPosition() > 175){
            direction = "center";
            FORWARD_TIME = 0.0;

            robot.fr.setPower(-SEARCH_SPEED);
            robot.fl.setPower(SEARCH_SPEED);
            robot.br.setPower(-SEARCH_SPEED);
            robot.bl.setPower(SEARCH_SPEED);
            while (opModeIsActive() && (runtime.seconds() < 0.6)) {
                telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.
            }
            robot.fr.setPower(0);
            robot.fl.setPower(0);
            robot.br.setPower(0);
            robot.bl.setPower(0);
        }

        //twist left
        while (opModeIsActive() && detector.getXPosition() < 330 && direction != "center"  && detector.getXPosition() != 0) {
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
        while (opModeIsActive() && detector.getXPosition() > 400 && direction != "left" && direction != "center"  && detector.getXPosition() != 0) {
            robot.fr.setPower(SEARCH_SPEED);
            robot.fl.setPower(-SEARCH_SPEED);
            robot.br.setPower(SEARCH_SPEED);
            robot.bl.setPower(-SEARCH_SPEED);

            twistTime = runtime.seconds();
            direction = "right";
            craterTwistTime = -twistTime;
            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.

        }

        detector.disable();


        if(direction == "left") {
            TURN_LEFT_TIME = 0.14;
            CUBE_PUSH_TIME = 1.1;
            FORWARD_MOVE_TIME = 0.7;

        }
        if(direction == "center") {
            TURN_LEFT_TIME = 0.34;
            CUBE_PUSH_TIME = 1.1;
            FORWARD_MOVE_TIME = 0.9;

        }
        if(direction == "right") {
            TURN_LEFT_TIME = 0.67;
            CUBE_PUSH_TIME = 1;
            FORWARD_MOVE_TIME = 1.5;

        }

//move forward to knock away cube
        robot.fr.setPower(FORWARD_SPEED);
        robot.fl.setPower(FORWARD_SPEED);
        robot.br.setPower(FORWARD_SPEED);
        robot.bl.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < CUBE_PUSH_TIME)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

//back up

        robot.fr.setPower(-FORWARD_SPEED);
        robot.fl.setPower(-FORWARD_SPEED);
        robot.br.setPower(-FORWARD_SPEED);
        robot.bl.setPower(-FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < BACKUP_DISTANCE)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

//turn left


        robot.fr.setPower(-FORWARD_SPEED);
        robot.fl.setPower(FORWARD_SPEED);
        robot.br.setPower(-FORWARD_SPEED);
        robot.bl.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < TURN_LEFT_TIME)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.fr.setPower(0);
        robot.fl.setPower(0);
        robot.br.setPower(0);
        robot.bl.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

//move forward
        robot.fr.setPower(FORWARD_SPEED);
        robot.fl.setPower(FORWARD_SPEED);
        robot.br.setPower(FORWARD_SPEED);
        robot.bl.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < FORWARD_MOVE_TIME)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //turn left
        robot.fr.setPower(-FORWARD_SPEED);
        robot.fl.setPower(FORWARD_SPEED);
        robot.br.setPower(-FORWARD_SPEED);
        robot.bl.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //strafe left into wall to line up robot
        robot.fr.setPower(-STRAFE_SPEED);
        robot.fl.setPower(STRAFE_SPEED);
        robot.br.setPower(STRAFE_SPEED);
        robot.bl.setPower(-STRAFE_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        //Full speed ahead to depot!
        robot.fr.setPower(FORWARD_SPEED);
        robot.fl.setPower(FORWARD_SPEED);
        robot.br.setPower(FORWARD_SPEED);
        robot.bl.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.fr.setPower(0);
        robot.fl.setPower(0);
        robot.br.setPower(0);
        robot.bl.setPower(0);

        //drop marker and pause
        robot.marker.setPosition(0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.marker.setPosition(1);

        //Full speed ahead into the crater!!! YAY!
        robot.fr.setPower(-FORWARD_SPEED);
        robot.fl.setPower(-FORWARD_SPEED);
        robot.br.setPower(-FORWARD_SPEED);
        robot.bl.setPower(-FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.8)) {
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
