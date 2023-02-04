package org.firstinspires.ftc.teamcode.ablaze.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

import org.firstinspires.ftc.teamcode.ablaze.common.Navigation;
//This program will:
//Read signal sleeve
//Place preloaded cone to closest high junction (TBD)
//Park robot in tile based on the sleeve reading
//Plane (1 Dot): Place robot in 1 diagonal left tile
//Logo (2 Dot): Place robot in 1 forward tile
//Jacket (3 Dot): Place robot in 1 diagonal right tile

@Autonomous
public class AblazeAuto extends LinearOpMode {
    AblazeRobot robot = new AblazeRobot();
    Navigation nav = new Navigation();
    AblazeTFOD tfod = new AblazeTFOD();
    private int signalZone = 0;
    private DcMotor verticalSlideMotor;
    private Servo clawServo;
    private double clawOpenPos = 0.5;
    private double clawClosePos = 1.0;
    private double motorPower;

    /*
    POS KEY:
    1: Top left corner ("A5")
    2: Bottom left corner ("A1")
    3: Top right corner ("E5")
    4: Bottom right corner ("E1")
    */

    @Override
    public void runOpMode() {
        //Notify init start
        telemetry.addData("Status", "Initializing");
        telemetry.update();


        //Set motor power
        motorPower = robot.getDefaultPower();

        // Initialize Hardware
        robot.initialize(hardwareMap);
        nav.initialize(hardwareMap, robot);
        tfod.initialize(hardwareMap, robot);
        verticalSlideMotor = robot.getVerticalSlideMotor();
        clawServo = robot.getClawServo();

        //Notify init end
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //Detect + park into signal zone
        //CHANGE ENCODER POSITIONS
        signalZone = tfod.detectElement(this);
        sleep(1000);

        telemetry.addData("the signal is",":"+signalZone);
        telemetry.update();
        if (signalZone == 2) {
            nav.encoderDrive(1000, "forward");
        } else if (signalZone == 1) {
            nav.encoderDrive(100, "forward");
            nav.encoderDrive(950, "left");
            nav.encoderDrive(1200, "forward");
            nav.encoderDrive(1000, "right");
            nav.encoderDrive(1000, "forward");
        } else if(signalZone == 3) {
            nav.encoderDrive(300, "forward");
            nav.encoderDrive(1100, "right");
            nav.encoderDrive(900, "forward");
            nav.encoderDrive(900, "left");
            nav.encoderDrive(1000, "forward");
        }
        else {
            telemetry.addData("No find", "");
            telemetry.update();
        }

        //Set Teleop init state
        clawServo.setPosition(clawOpenPos);

        /*
        //Phase 1: Setup - get scissors + arm ready
        clawServo.setPosition(clawClosePos);
        moveLinearSlides(1); //will sleep for 500 ms before next command
        sleep(500);

        //Phase 2: Auto - Detect signal zone, deliver pre-load cone, park
        //signalZone = tfod.detectElement();
        if(startPos == 2 || startPos == 3){
            nav.turn(90);
            nav.moveForward(22);
            nav.turn(-90);
            nav.moveForward(22);
            nav.turn(45);
            moveLinearSlides(2); //will sleep for 500 ms before next command
            nav.moveForward(2);
            clawServo.setPosition(clawOpenPos);
            sleep(500);
            nav.moveBackward(2);
            nav.turn(-45);
            nav.turn(-90);
            if(signalZone == 2){
                nav.moveForward(24);
            }
            else if(signalZone == 3){
                nav.moveForward(43);
            }

            //Move arm to teleop init pos
            moveLinearSlides(0); //will sleep for 500 ms before next command
        }
        else if(startPos == 1 || startPos == 4){
            nav.turn(-90);
            nav.moveForward(22);
            nav.turn(90);
            nav.moveForward(22);
            nav.turn(-45);
            moveLinearSlides(2); //will sleep for 500 ms before next command
            sleep(500);
            nav.moveForward(2);
            clawServo.setPosition(clawOpenPos);
            nav.moveBackward(2);
            nav.turn(45);
            nav.turn(90);
            if(signalZone == 2){
                nav.moveForward(24);
            }
            else if(signalZone == 3){
                nav.moveForward(43);
            }

            //Move arm to teleop init pos
            moveLinearSlides(0); //will sleep for 500 ms before next command
        }
        */
    }

    public void moveLinearSlides(int ticks){
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor.setTargetPosition(ticks);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(motorPower);
        while (verticalSlideMotor.isBusy()) {
            telemetry.addData("LFT, RFT", "Running to %7d", ticks);
            telemetry.addData("LFP, RFP", "Running at %7d",
                    verticalSlideMotor.getCurrentPosition()
            );
            telemetry.update();
        }
        verticalSlideMotor.setPower(motorPower);
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}


