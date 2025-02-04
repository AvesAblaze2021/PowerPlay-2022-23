package org.firstinspires.ftc.teamcode.ablaze.autonomous;
import android.sax.Element;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

import java.util.List;

public class AblazeTFOD {
    private static final String TFOD_MODEL_ASSET = "model_20221128_154930.tflite";
    private static final String[] LABELS = {
            "Fire",
            "Jacket",
            "Plane"

    };


    private static final String VUFORIA_KEY =
            "ASgJ37P/////AAABmeZUclRtJkpCrvmEuygWpAwPNksTf+k3zPsoV573qLI1dZpk" +
                    "SHF0eO5ETw9kC0ZvI1ft81RiXDQTvuQ2Nmc5NSjB7TaBkG9QKjeCVbzCCXiAg" +
                    "Gu8DmBF/Xg/7cHcCQqzVHc7RCsbId+BxAJMgz9Yx6pFvkYvGSz/1Fp2KJNw1O3Sg" +
                    "ps/g1Gc0BEln6vpnCjaBouBPNWzEh7vArEih81j0Ek8obttmiNEJ0qKbyefbxWzg" +
                    "ptxEQSyAIqqpNyWFSFngPGYFmhTV7cDahWGi46Jr0OYjJTrPamacGgY4XA8B5UH" +
                    "HLx7BITqp6j3EaZP5VivlYpb3sZLhoKxpwoNgg2E88hcfHkDl28FxLnlrnbMJIL9";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private AblazeRobot ablazeRobot;
    private Recognition element = null;
    private ElapsedTime runtime = new ElapsedTime();
    private int timeout = 1000;

    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();



    public int detectElement(LinearOpMode l) {
        // Comment out multiple try... just trying only once
        // Keep trying until an object is detected or time is expired
        //runtime.reset();
        //while (runtime.milliseconds() < timeout) {
        List<Recognition> recognitions = tfod.getRecognitions();
          while(runtime.milliseconds() < timeout) {
              if (recognitions != null) {
                  for (Recognition recognition : recognitions) {
                      if (recognition.getLabel().equals("Plane")) {
                          element = recognition;
                          return 1;
                      } else if (recognition.getLabel().equals("Fire")) {
                          element = recognition;
                          return 2;
                      } else if (recognition.getLabel().equals("Jacket")) {
                          element = recognition;
                          return 3;
                      }
                  }
              }
          }
        //}
        return -1;
    }
    private void initializeVuforia(){
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = ablazeRobot.getWebCam();
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initializeTfod(HardwareMap hwMap) {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfodParameters.inputSize = 320;
        tfodParameters.isModelTensorFlow2 = true;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public void initialize(HardwareMap hwMap, AblazeRobot ablazeRobot){
        this.ablazeRobot = ablazeRobot;
        initializeVuforia();
        initializeTfod(hwMap);
        if(tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }



}

