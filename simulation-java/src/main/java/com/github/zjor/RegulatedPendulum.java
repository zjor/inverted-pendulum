package com.github.zjor;

import processing.core.PApplet;

public class RegulatedPendulum extends PApplet {

    public static final int WIDTH = 800;
    public static final int HEIGHT = 600;

    static final int CART_WIDTH = 50;
    static final int CART_HEIGHT = 30;

    public static final double h = 0.01;
    public static final double l = 1;
    public static final double gravity = 9.8;

    /**
     * Pendulum state parameters
     */
    private double th, dth;

    /**
     * Cart state parameters
     */
    private double x, v;

    @Override
    public void settings() {
        size(WIDTH, HEIGHT);
    }

    @Override
    public void setup() {
        ellipseMode(CENTER);
        textAlign(LEFT, TOP);
        th = Math.PI / 6;
        dth = 0.0;
        x = 0.0;
        v = 0.0;
    }

    @Override
    public void draw() {

        integrate();

        fill(255);
        rect(0, 0, WIDTH, HEIGHT);

        fill(0);

        float y0 = HEIGHT / 2;
        double tx = x * 50 + WIDTH / 2;
        float x1 = (float) (tx + l * Math.sin(th) * 150);
        float y1 = (float) (y0 - l * Math.cos(th) * 150);

        line(0, y0, WIDTH, y0);
        fill(150);
        rect((float) tx - CART_WIDTH / 2, HEIGHT / 2 - CART_HEIGHT / 2, CART_WIDTH, CART_HEIGHT);

        ellipse((float) tx, y0, 10, 10);
        ellipse(x1, y1, 10, 10);

        line((float) tx, y0, x1, y1);

        text("Cart:", 10, 10);
        text("\tx:" + x, 16, 26);
        text("\tv:" + v, 16, 42);
        text("\tth:" + th, 16, 58);


    }

    private void integrate() {

        double Kp = 45.0;
        double Kd = 18.0;

        double xKp = 2.0;
        double xKd = 5.0;

        double control = Kp * th + Kd * dth + xKp * (x  - 3) + xKd * v;

        th += dth * h;
        dth += dTh(th, control) * h;

        v += control * h;
        x += v * h;
    }


    private double dTh(double th, double control) {
        return (Math.sin(th) * gravity - control * Math.cos(th)) / l;
    }

    public static void main(String[] args) {
        PApplet.main(RegulatedPendulum.class.getName());
    }
}
