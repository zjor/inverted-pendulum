package com.github.zjor;

import lombok.Getter;
import lombok.Setter;
import processing.core.PApplet;
import processing.event.MouseEvent;

public class App extends PApplet {

    public static final int WIDTH = 800;
    public static final int HEIGHT = 600;

    static final int CART_WIDTH = 50;
    static final int CART_HEIGHT = 30;

    static final long TIME_THRESHOLD = 10;

    public static final double h = 0.1;
    public static final double l = 150;
    public static final double gravity = 9.8;

    /**
     * Pendulum state variables
     */
    private double th, dth;

    private Cart cart;

    @Override
    public void settings() {
        size(WIDTH, HEIGHT);
    }

    @Override
    public void setup() {
        ellipseMode(CENTER);
        textAlign(LEFT, TOP);
        th = 0.5;
        dth = 0.0;

        cart = new Cart(WIDTH / 2, 1.0);
    }

    @Override
    public void draw() {

        double th1 = th + dth * h;
        double dth1 = dth + dTh(th) * h;


        th = th + (dth + dth1) * h / 2;
        dth = dth + (dTh(th) + dTh(th1)) * h / 2;

        if (isCartFree) {
            double dz1 = cart.getV() + dZ(th1) * h;
            cart.setX(cart.getX() + (cart.getV() + dz1) * h / 2);
            cart.setV(cart.getV() + (dZ(th) + dZ(th1)) * h / 2);
        }

        fill(255);
        rect(0, 0, WIDTH, HEIGHT);

        fill(0);

        float y0 = HEIGHT / 2;
        float x1 = (float) (cart.getX() + l * Math.sin(th));
        float y1 = (float) (y0 - l * Math.cos(th));

        line(0, y0, WIDTH, y0);
        cart.draw();
        ellipse((float) cart.getX(), y0, 10, 10);
        ellipse(x1, y1, 10, 10);

        line((float) cart.getX(), y0, x1, y1);

        text("Cart:", 10, 10);
        text("\tx:" + cart.getX(), 16, 26);
        text("\tv:" + cart.getV(), 16, 42);
        text("\ta:" + cart.getA(), 16, 58);

        if (isDragging && !isCartFree) {
            cart.updateX(mouseX - cart.getX());
        }


    }

    private boolean isDragging;
    private boolean isCartFree = true;

    private int mouseX;

    @Override
    public void mousePressed(MouseEvent event) {
        isDragging = true;
        mouseX = event.getX();
        isCartFree = !cart.isInside(event.getX(), event.getY());
    }

    @Override
    public void mouseDragged(MouseEvent event) {
        mouseX = event.getX();
    }

    @Override
    public void mouseReleased(MouseEvent event) {
        isDragging = false;
        isCartFree = true;
    }

    private double dTh(double th) {
        if (isCartFree) {
            return (Math.sin(th) * gravity - dZ(th) * Math.cos(th)) / l;
        } else {
            return (Math.sin(th) * gravity - cart.getA() * Math.cos(th)) / l;
        }
    }

    private double dZ(double th) {
        return - 0.2 * gravity * Math.sin(2.0 * th) / 2;
    }

    public static void main(String[] args) {
        PApplet.main(App.class.getName());
    }

    class Cart {
        private final float y = HEIGHT / 2;

        @Getter
        @Setter
        private double x, v, a, m;

        private long time;

        public Cart(double x, double m) {
            this.x = x;
            this.m = m;
            this.v = 0;
            this.a = 0;
            time = System.currentTimeMillis();
        }

        public void draw() {
            fill(150);
            rect((float) x - CART_WIDTH / 2, y - CART_HEIGHT / 2, CART_WIDTH, CART_HEIGHT);
        }

        public boolean isInside(int x, int y) {
            int x0 = (int) (this.x - CART_WIDTH / 2);
            int x1 = (int) (this.x + CART_WIDTH / 2);
            int y0 = (int) (this.y - CART_HEIGHT / 2);
            int y1 = (int) (this.y + CART_HEIGHT / 2);

            return (x >= x0 && x <= x1 && y >= y0 && y <= y1);
        }

        public void updateX(double dx) {
            x += dx;

            long now = System.currentTimeMillis();

            if (now - time > TIME_THRESHOLD) {
                double dt = 1.0 * (now - time) / 100;
                a = (dx / dt - v) / dt;
                v = dx / dt;
                time = now;
            }

        }

    }

}
