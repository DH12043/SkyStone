package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name= "TapeMeasureAutonomous", group= "None")
public class TapeMeasureAutonomous extends OpMode {

    DcMotor TapeMeasureMotor;

    private double startTapeTime;
    private double currentTapeTime;

    public void init() {
        msStuckDetectStart = 300000;
        TapeMeasureMotor = hardwareMap.dcMotor.get("TapeMeasureMotor");
    }

    public void start() {
        startTapeTime = getRuntime();
    }

    public void loop() {
        currentTapeTime = getRuntime();
        if (currentTapeTime - startTapeTime < .4) {
            TapeMeasureMotor.setPower(.6);
        }
        else {
            TapeMeasureMotor.setPower(0);
        }
    }
}
