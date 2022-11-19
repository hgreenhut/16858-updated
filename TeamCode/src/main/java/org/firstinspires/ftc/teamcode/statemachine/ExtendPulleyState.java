package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class ExtendPulleyState extends State {

    DcMotor pulley;
    Servo leftHand;
    Servo rightHand;
    boolean on = false;

    private double Power;
    private String Movement;

    private State NextState;

    private double Time;
    ElapsedTime mRuntime = new ElapsedTime();


    public ExtendPulleyState(StateMachine statemachine, double time, double power, ArrayList<DcMotor> motor, Servo arm, Servo clamp, String direction) {
        super(statemachine);
        Time = time;
        pulley = motor.get(4);
        leftHand = arm;
        rightHand = clamp;
        Power = power;
        mRuntime.reset();
        Movement = direction;

    }

    public void setNextState(State state) {
        NextState = state;

    }

    @Override
    public void start() {
        //  this.update();
        mRuntime.reset();
        pulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    @Override
    public void update() {


        while (mRuntime.seconds() < Time) {
            if (Movement == "extend") {
                pulley.setPower(Power);
            } else {
                pulley.setPower(-Power);
            }

        }


        //on = false;
        pulley.setPower(0);

        this.stop();
    }

    @Override
    void stop() {

    }


    @Override
    void initialize() {

    }

    public boolean getOn(){
        return on;
    }
}