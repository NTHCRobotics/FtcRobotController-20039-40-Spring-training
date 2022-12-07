package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class UseString extends OpMode {

    @Override

    public void init(){
        String myName = "Shruti Joshi";
        int myGrade = 10;
        telemetry.addData("Hello", myName);
        telemetry.addData("My grade level is", myGrade);
    }

    @Override

    public void loop(){

        int x = 5;
        //x is visible here
        {
            int y = 4;
            //x and y are visible here
        }
    //only x is visible here
    }

}
