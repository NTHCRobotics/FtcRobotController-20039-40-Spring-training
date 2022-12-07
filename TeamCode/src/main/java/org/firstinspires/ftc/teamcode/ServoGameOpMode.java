package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard5;

@TeleOp
public class ServoGameOpMode extends OpMode{
    ProgrammingBoard5 board = new ProgrammingBoard5();
    @Override
    public void init(){
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        if (gamepad1.a){
            board.setServoPosition(1.0);
        }
    }
}
