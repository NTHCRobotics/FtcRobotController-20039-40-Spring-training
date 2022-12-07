package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class ProgrammingBoard4 {
    private DigitalChannel touchSensor;
    private DcMotor motor;
    private double ticksPerRotation;

    public void init(HardwareMap hwMap){
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        motor = hwMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
    }
    public boolean isTouchSensorPressed(){
        return !touchSensor.getState();
    }

    public void setMotorSpeed(double speed){
        motor.setPower(speed);
    }
    public double getMotorRotations(){
        return motor.getCurrentPosition()/ticksPerRotation;
    }

    public boolean zeroPowerBehavior(){

    }

}
