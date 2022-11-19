package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ClaspState extends State {

    Servo arm;
    ElapsedTime mRuntime = new ElapsedTime();
    Double time;
    double Power;
    String Movement;
    double pos;
    // private StateMachine.State NextState;
    public ClaspState(StateMachine statemachine, Servo clasp, double sec, double position){
        super(statemachine);
        arm = clasp;
        time = sec;
        mRuntime.reset();

        pos = position;
    }
    @Override
    public void start() {
        mRuntime.reset();

    }

    @Override
    public void update() {

//        if(arm.getPosition() == 0)
//            arm.setPosition(1);
//        else
        while (mRuntime.seconds()<=time) {
            arm.setPosition(pos);

        }
        //(1000);
        this.stop();



    }

    @Override
    void stop() {

    }

    @Override
    void initialize() {

    }

}