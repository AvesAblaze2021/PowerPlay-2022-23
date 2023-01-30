package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class LinearSlideTest extends LinearOpMode{
  AblazeRobot robot = new AblazeRobot();
  int lastPos = 0; //Used for rel encoder algorithm
  double motorPower = 0.3;
  private DcMotor vertical;

  @Override
  public void runOpMode(){
    robot.initialize(hardwareMap);
    vertical = robot.getVerticalSlideMotor();

    waitForStart();

    moveSlideEncoderAbs(vertical, 1000);
    sleep(2000);
    moveSlideEncoderAbs(vertical, 500);
  }
  
  //Regular timed algorithm
  public void moveSlideTime(DcMotor slideMotor, double power, int time){
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    while(true){
      slideMotor.setPower(power);
      sleep(time);
      slideMotor.setPower(0);
      sleep(2000);
      slideMotor.setPower(-power);
      sleep(time / 2);
      slideMotor.setPower(0);
      sleep(2000);
    }

  }
  
  //Relative algorithm that resets encoder after each move
  public void moveSlideEncoderRel(int ticks){
    //Encoder setup - adjust tick value w/ last pos
    int newTicks = Math.abs(ticks - lastPos);
    vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    //Starting motor
    vertical.setTargetPosition(newTicks);
    vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    vertical.setPower(0.2);
    
    //Encoder loop
    while(opModeIsActive() && vertical.isBusy()){
      continue;
    }
    
    //End motion + store current tick value before reset
    vertical.setPower(0);
    lastPos = vertical.getCurrentPosition();
    vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }
  
  //Absolute positioning algorithm without encoder resets
  public void moveSlideEncoderAbs(int ticks){
    vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    vertical.setTargetPosition(ticks);
    vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    vertical.setPower(motorPower);
    while (opModeIsActive() && vertical.isBusy()) {
      telemetry.addData("LFT, RFT", "Running to %7d", ticks);
      telemetry.addData("LFP, RFP", "Running at %7d",
              vertical.getCurrentPosition()
      );
      telemetry.update();
    }
    vertical.setPower(0.0);
  }
    
}
