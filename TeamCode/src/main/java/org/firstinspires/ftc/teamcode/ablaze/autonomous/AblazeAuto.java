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
    private Servo clawServo;
    private double clawOpenPos = 0.0;
    private double clawClosePos = 1.0;
    private double rotDeliverPos = 0.85;
    private double rotPickupPos = 0.21;
    private final double motorPower = 0.3;

    @Override
    public void runOpMode() {
        // Initialize Hardware
        robot.initialize(hardwareMap);
        nav.initialize(hardwareMap);
        //tfod.initialize(hardwareMap, robot);
        verticalSlideMotor = robot.getVerticalSlideMotor();
        rotationServo = robot.getRotationServo();
        clawServo = robot.getClawServo();

        //Init state set after start b/c of size limitations

        //Notify init end
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        //Phase 1: Setup - get scissors + arm ready
        clawServo.setPosition(clawClosePos);
        moveLinearSlides(1);
        rotationServo.setPosition(rotPickupPos);
        sleep(500);

        //Phase 2: Auto - Detect signal zone, deliver pre-load cone, park
        //signalZone = tfod.detectElement();
        nav.turn(90);
        nav.moveForward(22);
        nav.turn(-90);
        nav.moveForward(22);
        nav.turn(45);
        moveLinearSlides(2);
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
        moveLinearSlides(0); //Move arm to teleop init pos
    }

    public void moveLinearSlides(int level){
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor.setTargetPosition(level_ticks[level]);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(motorPower);
        while (verticalSlideMotor.isBusy()) {
            telemetry.addData("LFT, RFT", "Running to %7d", level_ticks[level]);
            telemetry.addData("LFP, RFP", "Running at %7d",
                    verticalSlideMotor.getCurrentPosition()
            );
            telemetry.addData("level", level);
            telemetry.update();
        }
        sleep(500);
    }
}

