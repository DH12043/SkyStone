package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "CheckLoopTiming")
public class CheckLoopTiming extends OpMode {

    private double _lastCurrentTime;

    @Override
    public void init () {
        _lastCurrentTime = getRuntime();
    }

    @Override
    public void loop() {

        double currentTime = System.nanoTime();
        double diff = currentTime - _lastCurrentTime;
        telemetry.addData("Milliseconds Per Loop","LoopValue/1000000 %32f", diff);
        _lastCurrentTime = currentTime;
    }
}
