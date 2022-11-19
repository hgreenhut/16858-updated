package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class ColorSenseStopState extends State {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    String cval;
    State NextState;
    ColorSensor cs1;
    String dir;
    double pow;
    int red;
    public ColorSenseStopState(StateMachine stateMachine, ArrayList<DcMotor> motor, ColorSensor colorSensor, String color, double power, String direction){
        super(stateMachine);
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        cs1 = colorSensor;
        cval = color;
        pow = power;
        dir = direction;
        //red = cs1.red();
    }

    public void update(){

        if(cval.equals("red")){
            if(dir.equals("forward")){
                leftBack.setPower(pow);
                leftFront.setPower(pow);
                rightBack.setPower(pow);
                rightFront.setPower(pow);
            }
            else if(dir.equals("backward")){
                leftBack.setPower(-pow);
                leftFront.setPower(-pow);
                rightBack.setPower(-pow);
                rightFront.setPower(-pow);
            }

            if(cs1.red()> 1000 && cs1.red()>cs1.blue() && cs1.red()>cs1.green()){
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                this.startNextState();
            }

        }
        else if(cval.equals("blue")){
            if(dir.equals("forward")){
                leftBack.setPower(pow);
                leftFront.setPower(pow);
                rightBack.setPower(pow);
                rightFront.setPower(pow);
            }
            else if(dir.equals("backward")){
                leftBack.setPower(-pow);
                leftFront.setPower(-pow);
                rightBack.setPower(-pow);
                rightFront.setPower(-pow);
            }

            if(cs1.blue()> 1000 && cs1.blue()>cs1.red() && cs1.blue()>cs1.green()){
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                this.startNextState();
            }

        }
    }

    @Override
    void stop() {
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        this.startNextState();
    }


    @Override
    void initialize() {

    }

    public int getColor(){

        return cs1.red();

    }

    @Override
    public void start() {

    }
}
