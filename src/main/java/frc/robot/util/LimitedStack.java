package frc.robot.util;

import java.util.ArrayDeque;

public class LimitedStack<T> extends ArrayDeque<T>{

    private final int maxSize;
    int count = 0;

    public LimitedStack(int maxSize){
        super(maxSize);
        this.maxSize = maxSize;
    }

    @Override
    public void push(T e){
        count++;
        if(count > maxSize){
            //super.getLast();
            count--;
        }
        super.push(e);
    }

}
