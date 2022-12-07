package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Game_pad extends OpMode {
    @Override

    public void init(){
    }

    @Override
    public void loop(){
        double speedForward = -gamepad1.left_stick_y/2.0;
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("A button", gamepad1.a);
        telemetry.addData("speed Forward", speedForward);

        //right stick of the gamepad
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);

        //show that button b is pressed on gamepad1
        telemetry.addData("B button", gamepad1.b);

        double difference = gamepad1.left_stick_y - gamepad1.right_stick_y;

        double sum = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.right_stick_y;
    }



}
