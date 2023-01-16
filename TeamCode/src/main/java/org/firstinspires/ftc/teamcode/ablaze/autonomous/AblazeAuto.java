package org.firstinspires.ftc.teamcode.ablaze.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;
import org.firstinspires.ftc.teamcode.ablaze.common.RRNavigation;

@Autonomous
public class AblazeAuto extends LinearOpMode {
    AblazeRobot robot = new AblazeRobot();
    RRNavigation nav = new RRNavigation();
    AblazeTFOD tfod = new AblazeTFOD();
    private final int[] vertical_level_ticks = {100, 1200, 3800}; //Tick values for every level
    private int signalZone = 2; //temporary
    private DcMotor verticalSlideMotor;
    private Servo rotationServo;
    private Servo scissorServo;
    private double pickupPos = 0.5;
    private double dropPos = 0.4;
    private final double motorPower = 0.3;

    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);
        nav.initialize(hardwareMap);
        //tfod.initialize(hardwareMap, robot);

        // Initialize Robot
        verticalSlideMotor = robot.getVerticalSlideMotor();
        rotationServo = robot.getRotationServo();
        scissorServo = robot.getScissorServo();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        //Phase 1: Setup - get scissors + arm ready
        scissorServo.setPosition(pickupPos);
        moveLinearSlides(verticalSlideMotor, vertical_level_ticks, 1);
        rotationServo.setPosition(0.21);
        sleep(1000);

        //Phase 2: Auto - Detect signal zone, deliver pre-load cone, park
        //signalZone = tfod.detectElement();
        nav.turn(90);
        nav.moveForward(22);
        nav.turn(-90);
        nav.moveForward(22);
        nav.turn(45);
        moveLinearSlides(verticalSlideMotor, vertical_level_ticks, 2);
        nav.moveForward(2);
        scissorServo.setPosition(dropPos);
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
    }

    public void moveLinearSlides(DcMotor slideMotor, int[] level_ticks, int level){
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setTargetPosition(level_ticks[level]);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(motorPower);
        while (slideMotor.isBusy()) {
            telemetry.addData("LFT, RFT", "Running to %7d", level_ticks[level]);
            telemetry.addData("LFP, RFP", "Running at %7d",
                    slideMotor.getCurrentPosition()
            );
            telemetry.addData("level", level);
            telemetry.update();
        }
        sleep(500);
    }
}

