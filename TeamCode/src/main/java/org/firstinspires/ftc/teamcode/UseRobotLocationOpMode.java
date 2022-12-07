package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UseRobotLocationOpMode extends OpMode{
    RobotLocation robotLocation = new RobotLocation(0);
    double x;

    @Override
    public void init(){
        robotLocation.setAngle(0);
    }

    double getAngle(){
        return getAngle();
    }

    double getX(){
        return x;
    }

    public void changeX(double change){
    }

    public void setX(double X){
    }


    @Override
    public void loop(){
        if (gamepad1.a){
            robotLocation.turn(0.1);
        }
        else if (gamepad1.b){
            robotLocation.turn(-0.1);
        }

        if (gamepad1.dpad_left){

        }
        telemetry.addData("Location", robotLocation);
        telemetry.addData("Heading", robotLocation.getHeading());
    }
}
