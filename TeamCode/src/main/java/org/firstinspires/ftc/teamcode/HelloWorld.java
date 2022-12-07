package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class HelloWorld extends OpMode {
    /**
     * This is called when the driver presses INIT
     */
    @Override
    public void init() {
        //this sends to the driver station
        telemetry.addData("Hello","My Name is Shruti");
    }

    /**
     * This is called repeatedly while OpMode is playing
     */
    @Override
    public void loop(){
        //intentionally left blank
    }
}


