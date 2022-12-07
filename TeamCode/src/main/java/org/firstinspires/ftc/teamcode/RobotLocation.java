package org.firstinspires.ftc.teamcode;

public class RobotLocation {
    double angle;

    public RobotLocation(double angle){
        this.angle = angle;

    }

    public double getHeading(){
        double angle = this.angle;
        while(angle > 180){
            angle-= 360;

        }
        while (angle < -180){
            angle += 360;

        }
        return angle;
    }

    @Override
    public String toString(){
        return "RobotLocation: angle (" + angle + ")";
    }

    public void turn(double angleChange){
        angle += angleChange;

    }

    public void setAngle(double angle){
        this.angle = angle;
    }
}
