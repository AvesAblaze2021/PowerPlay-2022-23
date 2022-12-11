package org.firstinspires.ftc.teamcode.ablaze.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class AblazeAuto extends LinearOpMode {
    AblazeRobot robot = new AblazeRobot();

    //AblazeTFOD..java tfod = new AblazeTFOD..java();
    private DcMotor verticalSlideMotor;
    private ElapsedTime runtime = new ElapsedTime();
    String location = "A1";

    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);
        verticalSlideMotor = robot.getVerticalSlideMotor();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();
        telemetry.addData("Mode", "running");
        telemetry.update();
        moveLinearSlides(.2, 300);
    }

    private void moveLinearSlides(double speed, int ticks) {
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor.setTargetPosition(ticks);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(Math.abs(speed));
        while (opModeIsActive() && verticalSlideMotor.isBusy()) {
            telemetry.addData("LFT, RFT", "Running to %7d", ticks);
            telemetry.addData("LFP, RFP", "Running at %7d",
                    verticalSlideMotor.getCurrentPosition()
            );
            telemetry.update();
        }
        verticalSlideMotor.setPower(0.0);
    }
}

