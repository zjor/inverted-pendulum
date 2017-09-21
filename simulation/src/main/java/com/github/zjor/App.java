package com.github.zjor;

import processing.core.PApplet;

import java.util.function.Function;

public class App extends PApplet {

    public static final int WIDTH = 800;
    public static final int HEIGHT = 600;


    public static final double H = 0.2;
    public static final double L = 100;

    private double th, dth;

    @Override
    public void settings() {
        size(WIDTH, HEIGHT);
    }

    @Override
    public void setup() {
        ellipseMode(CENTER);

        th = 30.0;
        dth = 0.0;
    }

    @Override
    public void draw() {
        Function<Double, Double> f = th -> Math.sin(th * Math.PI / 180.0);

        double th1 = th + dth * H;
        double dth1 = dth + f.apply(th) * H;

        th = th + (dth + dth1) * H / 2;

        dth = dth + (f.apply(th) + f.apply(th1)) * H / 2;

        fill(255);
        rect(0, 0, WIDTH, HEIGHT);

        fill(0);
        float x0 = WIDTH / 2;
        float y0 = HEIGHT / 2;
        float x1 = (float) (x0 + L * Math.sin(th * Math.PI / 180.0));
        float y1 = (float) (y0 - L * Math.cos(th * Math.PI / 180.0));

        ellipse(x0, y0, 10, 10);
        ellipse(x1, y1, 10, 10);

        line(x0, y0, x1, y1);
    }

    public static void main(String[] args) {
        PApplet.main(App.class.getName());
    }

}
