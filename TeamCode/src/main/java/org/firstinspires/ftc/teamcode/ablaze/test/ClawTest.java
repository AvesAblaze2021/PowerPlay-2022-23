package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;
import org.firstinspires.ftc.teamcode.ablaze.teleop.AblazeTeleop;

//Teleop test program for testing attachments
@TeleOp
public class ClawTest extends OpMode {
    AblazeRobot robot = new AblazeRobot();

    private boolean isLoop = false;
    private Servo clawServo;
    private double clawOpenPos = 0.0;
    private double clawClosePos = 1.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Notify init start
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Initialize Hardware
        robot.initialize(hardwareMap);
        clawServo = robot.getClawServo();

        //Teleop's init state is set at the end of Autonomous

        //Notify init end
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        isLoop = true;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     *
     */
    @Override
    public void loop(){
        if(gamepad2.b){
          clawServo.setPosition(clawClosePos);
        }
        if(gamepad2.a){
          clawServo.setPosition(clawOpenPos);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        isLoop = false;
    }
}
