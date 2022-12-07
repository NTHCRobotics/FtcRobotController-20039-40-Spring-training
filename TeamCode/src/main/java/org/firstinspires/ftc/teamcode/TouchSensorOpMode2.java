package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard2;

@TeleOp
public class TouchSensorOpMode2 extends OpMode{
    ProgrammingBoard2 board = new ProgrammingBoard2();
    @Override
    public void init(){
        board.init(hardwareMap);
    }



    public String loop1(){
        if (board.isTouchSensorPressed()){
            return "Pressed";
        }
        return "Not pressed";
    }
    @Override
    public void loop(){

        telemetry.addData("Touch sensor", board.isTouchSensorPressed());
        telemetry.addData("Touch sensor", board.isTouchSensorReleased());
    }
}



