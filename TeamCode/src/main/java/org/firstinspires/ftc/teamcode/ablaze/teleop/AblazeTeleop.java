//Aves Ablaze 2021-22 Official States Teleop
package org.firstinspires.ftc.teamcode.ablaze.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;
import org.firstinspires.ftc.teamcode.ablaze.common.PracticeRobot;
@TeleOp(name="AblazeTeleop", group="AblazeRobot")
public class AblazeTeleop extends OpMode{
    PracticeRobot practiceRobot = new PracticeRobot();
    ElapsedTime runtime = new ElapsedTime();

    private boolean isLoop = false;

    //All possible States for all Attachments - change for Power Play
    private enum AttachmentState{
        START, //Initial State - gamepad monitoring occurs here
        CHECK, //Arm
        OPEN, //Arm Claw
        CLOSE, //Arm Claw
        ACTIVATE, //Carousel
        DEACTIVATE //Carousel
    };

    private AttachmentState attachState = AttachmentState.START;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize Robot
        practiceRobot.initialize(hardwareMap);

        // Use Encoder for Arm

        // Do not Use Encoder for Wheels

        // Send telemetry message to signify robot waiting;
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     * Change for Power Play
     */
    @Override
    public void loop(){
        isLoop = true;
        //Check all states
        switch(attachState){
            case START: //Initial state - all gamepad conditionals go here, leads to other States
                if(gamepad2.a){
                    attachState = AttachmentState.OPEN;
                }
                if(gamepad2.b){
                    attachState = AttachmentState.CLOSE;
                }
                if(gamepad2.x){
                    attachState = AttachmentState.ACTIVATE;
                }
                if(gamepad2.y){
                    attachState = AttachmentState.DEACTIVATE;
                }
                break;

            case CHECK: //Check if Arm has either reached target or if timeout has been reached
                        //Similar to while loop used in autonomous
                attachState = AttachmentState.START;
                break;

            case OPEN:
                attachState = AttachmentState.START;
                break;

            case CLOSE:
                attachState = AttachmentState.START;
                break;

            case ACTIVATE:
                attachState = AttachmentState.START;
                break;

            case DEACTIVATE:
                attachState = AttachmentState.START;
                break;
        }

        //State machine doesn't interfere with drivetrain code here
        driveWithTwoJoysticks();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        isLoop = false;
    }

    // drive with joysticks
    public void driveWithTwoJoysticks() {
        double px = gamepad1.left_stick_x;
        double py = -gamepad1.left_stick_y;
        double pa = (gamepad1.right_stick_x - gamepad1.right_stick_y);
        if (Math.abs(pa) < 0.03) pa = 0;
        double p1 = -px + py + pa;
        double p2 = px + py + pa;
        double p3 = -px + py - pa;
        double p4 = px + py - pa;
        if(gamepad1.left_bumper) {
            p1 /= 3;
            p2 /= 3;
            p3 /= 3;
            p4 /= 3;
        }
        if(gamepad1.right_bumper) {
            p1 /= 1.2;
            p2 /= 1.2;
            p3 /= 1.2;
            p4 /= 1.2;
        }
        else if (isLoop){
            p1 /= 1.6;
            p2 /= 1.6;
            p3 /= 1.6;
            p4 /= 1.6;
        }

        //sets the speed of the drive motors
        practiceRobot.getLeftBackDrive().setPower(p1);
        telemetry.addData("motor power of left back motor is", p1);
        practiceRobot.getLeftFrontDrive().setPower(p2);
        telemetry.addData("motor power of left front motor is", p2);
        practiceRobot.getRightFrontDrive().setPower(p3);
        telemetry.addData("motor power of Right front motor is", p3);
        practiceRobot.getRightBackDrive().setPower(p4);
        telemetry.addData("motor power of Right back motor is", p4);
    }
}
