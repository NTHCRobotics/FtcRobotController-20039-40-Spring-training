package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp()
public class Chapter_4 extends OpMode{

    @Override

    public void init(){

    }

    @Override

    public void loop(){

        //IfOpMode/IfElseOpMode
        if(gamepad1.left_stick_y < 0){
            telemetry.addData("Left stick", "is negative");
        }
        else{
            telemetry.addData("Left stick", " is postive");
        }

        telemetry.addData("Left stick y", gamepad1.left_stick_y);


        //IfOpMode2

        if(gamepad1.a){
            telemetry.addData("A button", "pressed");

        }

        //IfElseIfOpMode

        if (gamepad1.left_stick_y < -0.5){
            telemetry.addData("Left stick", " is negative and large");
        }
        else if (gamepad1.left_stick_y < 0){
            telemetry.addData("Left stick", " is negative and small");
        }
        else if(gamepad1.left_stick_y < 0.5){
            telemetry.addData("Left stick", " is positive and small");
        }
        else{
            telemetry.addData("Left stick", " is positive and large");
        }


        //turbo button

        if(gamepad1.a == false){


        }
    }


}
