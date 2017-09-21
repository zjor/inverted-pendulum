package com.github.zjor;

import processing.core.PApplet;

public class App extends PApplet {

    public static final int WIDTH = 800;
    public static final int HEIGHT = 600;


    public static final double h = 0.1;
    public static final double l = 100;
    public static final double gravity = 9.8;

    /**
     * Pendulum state variables
     */
    private double th, dth;

    /**
     * Mount state variables
     */
    private double z, dz;

    @Override
    public void settings() {
        size(WIDTH, HEIGHT);
    }

    @Override
    public void setup() {
        ellipseMode(CENTER);

        th = 0.5;
        dth = 0.0;
        z = WIDTH / 2;
        dz = 0.0;
    }

    @Override
    public void draw() {

        double th1 = th + dth * h;
        double dth1 = dth + dTh(th) * h;
        double dz1 = dz + dZ(th1) * h;

        th = th + (dth + dth1) * h / 2;
        dth = dth + (dTh(th) + dTh(th1)) * h / 2;


        z = z + (dz + dz1) * h / 2;
        dz = dz + (dZ(th) + dZ(th1)) * h / 2;


        fill(255);
        rect(0, 0, WIDTH, HEIGHT);

        fill(0);
        float x0 = WIDTH / 2;
        float y0 = HEIGHT / 2;
        float x1 = (float) (z + l * Math.sin(th));
        float y1 = (float) (y0 - l * Math.cos(th));

        ellipse((float) z, y0, 10, 10);
        ellipse(x1, y1, 10, 10);

        line((float) z, y0, x1, y1);

        line(0, y0, WIDTH, y0);
    }

    private double dTh(double th) {
        return (Math.sin(th) * gravity - dZ(th) * Math.cos(th)) / l;
    }

    private double dZ(double th) {
        return - gravity * Math.sin(2.0 * th) / 2;
    }

    public static void main(String[] args) {
        PApplet.main(App.class.getName());
    }

}
