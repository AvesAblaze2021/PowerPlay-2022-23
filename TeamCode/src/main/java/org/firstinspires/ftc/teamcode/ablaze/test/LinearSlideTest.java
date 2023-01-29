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
  int lastPos = 0; //Used for encoder algorithm
  double motorPower = 0.3;
  private DcMotor vertical;

  @Override
  public void runOpMode(){
    robot.initialize(hardwareMap);
    vertical = robot.getVerticalSlideMotor();

    waitForStart();

    moveSlideEncoderAbs(vertical, 50);
  }

  //Continiously prints current position of encoder
  public void keepGettingPos(DcMotor slideMotor){
    int pastPos = 0;
    while(true){
      int currentPos = slideMotor.getCurrentPosition();
      if(currentPos != pastPos){
        telemetry.addData("pos", currentPos);
        telemetry.update();
        pastPos = currentPos;
      }
    }
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
  
  //Option 2 algorithm described in pseudocode - see drive
  public void moveSlideEncoderRel(DcMotor slideMotor, int ticks){
    //Encoder setup - adjust tick value w/ last pos
    int newTicks = Math.abs(ticks - lastPos);
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //Starting motor
    slideMotor.setTargetPosition(newTicks);
    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideMotor.setPower(0.2);
    
    //Encoder loop
    while(opModeIsActive() && slideMotor.isBusy()){
      continue;
    }
    
    //End motion + store current tick value before reset
    slideMotor.setPower(0);
    lastPos = slideMotor.getCurrentPosition();
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  public void moveSlideEncoderAbs(DcMotor slideMotor, int ticks){ //USE THIS
    slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    slideMotor.setTargetPosition(ticks);
    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideMotor.setPower(motorPower);
    while (opModeIsActive() && slideMotor.isBusy()) {
      telemetry.addData("LFT, RFT", "Running to %7d", ticks);
      telemetry.addData("LFP, RFP", "Running at %7d",
              vertical.getCurrentPosition()
      );
      telemetry.update();
    }
    slideMotor.setPower(0.0);
  }
    
}
