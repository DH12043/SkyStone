package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


abstract class CheckLoopTiming extends OpMode {

    protected double _lastCurrentTime;

    @Override
    public void init () {
        _lastCurrentTime = getRuntime();
    }

    @Override
    public void loop() {

        double currentTime = System.nanoTime();
        double diff = currentTime - _lastCurrentTime;
        telemetry.addData("Milliseconds Per Loop","LoopValue/1000000 %7f", diff);
        _lastCurrentTime = currentTime;
    }
}
