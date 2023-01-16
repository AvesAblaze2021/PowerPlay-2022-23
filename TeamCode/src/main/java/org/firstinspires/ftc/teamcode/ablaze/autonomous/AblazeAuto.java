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
    private final int[] vertical_level_ticks = {100, 1500, 3500}; //Tick values for every level
    private final int[] horizontal_level_ticks = {0, 1000};
    private int signalZone = 2; //temporary
    private DcMotor verticalSlideMotor;
    private DcMotor horizontalSlideMotor;
    private Servo rotationServo;
    private Servo scissorServo;
    private Servo alignServo;
    String location = "A5";
    private int horizontal_level = 0; //0 = Start, 1 = Low, 2 = Medium, 3 = High
    private int vertical_level = 0;
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
        alignServo = robot.getAlignServo();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        //Phase 1: Init
        scissorServo.setPosition(0.5);
        vertical_level = 1;
        moveLinearSlides(verticalSlideMotor, vertical_level_ticks, vertical_level);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        rotationServo.setPosition(0.21);

        //signalZone = tfod.detectElement();
        nav.turn(90);
        nav.moveForward(22);
        nav.turn(-90);
        nav.moveForward(22);
        nav.turn(45);
        nav.moveForward(2);
        vertical_level += 1;
        moveLinearSlides(verticalSlideMotor, vertical_level_ticks, vertical_level);
        scissorServo.setPosition(0.4);
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
    }
}

