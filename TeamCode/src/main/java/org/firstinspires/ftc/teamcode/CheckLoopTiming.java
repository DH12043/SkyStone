package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "PowerSurgeTeleOp")
public class CheckLoopTiming extends OpMode {

    private double _lastCurrentTime;
    
    @Override
    public void init () {
        _lastCurrentTime = getRuntime();
    }
    
    @Override
    public void loop() {
        
        double currentTime = this.getRuntime();
        double diff = currentTime - _lastCurrentTime;
        telemetry.addData("Milliseconds Per Loop","LoopValue %.2f", diff);
        _lastCurrentTime = currentTime;
    }
}
