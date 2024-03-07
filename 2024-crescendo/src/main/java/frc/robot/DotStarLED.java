package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class DotStarLED {
    private Timer timer;
    private DotStarManager dsm;
    public enum AnimationType {
        BLINK,
        PULSE
    }

    public class LEDConfig {
        public AnimationType type;
        public Color color1;
        public Color color2;
        public double framerate;
        public double zoom;

        public LEDConfig setType(AnimationType type) {
            this.type = type;
            return this;
        }

        public LEDConfig setColor1(Color color1) {
            this.color1 = color1;
            return this;
        }

        public LEDConfig setColor2(Color color2) {
            this.color2 = color2;
            return this;
        }

        public LEDConfig setFramerate(double framerate) {
            this.framerate = framerate;
            return this;
        }

        public LEDConfig setZoom(double zoom) {
            this.zoom = zoom;
            return this;
        }
    }

    public DotStarLED() {
        timer = new Timer();
        timer.start();
    }

    public void update(LEDConfig config) {
        int frame = (int) (timer.get() * config.framerate);
        Color finalColor;
        switch (config.type) {
            case BLINK:
                if (frame % 2 == 0) {
                    finalColor = config.color1;
                } else {
                    finalColor = config.color2;
                }
                break;
            case PULSE:
                frame = frame % 512;
                double redUnit = (config.color2.getRed() - config.color1.getRed()) / 255.0;
                double greenUnit = (config.color2.getGreen() - config.color1.getGreen()) / 255.0;
                double blueUnit = (config.color2.getBlue() - config.color1.getBlue()) / 255.0;
                finalColor = new Color();
                if (frame < 256) {
                    finalColor.setRed(config.color1.getRed() + (int) (redUnit * frame));
                    finalColor.setGreen(config.color1.getGreen() + (int) (greenUnit * frame));
                    finalColor.setBlue(config.color1.getBlue() + (int) (blueUnit * frame));
                } else {
                    finalColor.setRed(config.color2.getRed() - (int) (redUnit * frame - 255));
                    finalColor.setGreen(config.color2.getGreen() - (int) (greenUnit * frame - 255));
                    finalColor.setBlue(config.color2.getBlue() - (int) (blueUnit * frame - 255));
                }
                break;
            default:
                finalColor = new Color();
                break;
        }

        dsm.fill(finalColor);
        dsm.show();
    }

    //Might be used later
    private class VirtualStrip {
        private DotStarManager realStrip;
        private int position;
        private int length;

        /**
         * Creates a "virtual strip" (a selection of LEDs) on a real strip of DotStars. Intended to be used to differentiate between physical strips when multiple strips are wired together.
         * @param realStrip
         * @param position
         * @param length
         */
        public VirtualStrip(DotStarManager realStrip, int position, int length) {
            this.realStrip = realStrip;
            this.position = position;
            this.length = length;
        }

        /**
         * Sets all LEDs in this virtual strip to turn off on the next {@code show()} run on the real strip.
         */
        public void clear() {
            fill(new Color());
        }

        /**
         * Sets the LED at index {@code i} on this virtual strip to turn {@code Color c} on the next {@code show()} run on the real strip.
         * 
         * @param i the zero-indexed ID of the LED
         * @param c the color to write
         */
        public void set(int i, Color c) {
            if (i >= length) {
                System.out.println(
                        "VirtualStrip.WARNING: You have tried to write to LED "
                                + i + ", which is not on this virtual DotStar strip!");
                return; // Keeps the line below the if-statement from running
            }
            realStrip.set(position + i, c);
        }

        public void fill(Color c) {
            for (int i = position; i < position + length; i++) {
                realStrip.set(i, c);
            }
        }
    }

    private class DotStarManager { // Class to do low-level SPI data management for the DotStars
        private Color[] strip;
        private SPI spi;

        /**
         * Create a manager for a strip of DotStars
         * @param length the length of the strip
         */
        public DotStarManager(int length) {
            strip = new Color[length];
            spi = new SPI(SPI.Port.kOnboardCS0); //TODO: Check that this port is correct.
            clear(); // Sets all elements to be new `Color()`s, so this doubles as a method to instantiate the array elements.
            show();
        }

        /**
         * Sets all LEDs to turn off on the next {@code show()}.
         */
        public void clear() {
            fill(new Color());
        }

        /**
         * Sets the LED at index {@code i} to turn {@code Color c} on the next {@code show()}.
         * @param i the zero-indexed ID of the LED
         * @param c the color to write
         */
        public void set(int i, Color c) {
            try {
                strip[i] = c;
            } catch (IndexOutOfBoundsException e) {
                System.out.println("DotStarManager.WARNING: You have tried to access LED "
                        + i + ", which is not on this DotStar strip!");
            }
        }

        /**
         * Writes all changes made by {@code set()} and {@code clear()} to the LED strip.
         */
        public void show() {
            spi.write(intToByteArray(new int[] { 0x00, 0x00, 0x00, 0x00 }), 4);
            for (int i = 0; i < strip.length; i++) {
                spi.write(intToByteArray(0xFF), 1);
                spi.write(intToByteArray(strip[i].getIntArray()), 3);
            }
            spi.write(intToByteArray(new int[] { 0xFF, 0xFF, 0xFF, 0xFF }), 4);
        }

        /**
         * Set all LEDs in the strip to be the {@code Color c} on the next {@code show()}.
         * @param c the color to set
         */
        public void fill(Color c) {
            for (Color d : strip) {
                d = c;
            }
        }

        private byte[] intToByteArray(int i) {
            return new byte[] { (byte) i };
        }

        private byte[] intToByteArray(int[] i) {
            byte[] b = new byte[i.length];
            for (int j = 0; j < i.length; j++) {
                b[j] = (byte) i[j];
            }
            return b;
        }
    }

    private class Color {
        private int red;
        private int green;
        private int blue;

        public Color() {
            red = 0;
            green = 0;
            blue = 0;
        }

        public Color(double red, double green, double blue) {
            this.red = (int) red;
            this.green = (int) green;
            this.blue = (int) blue;
        }

        public int getRed() {
            return red;
        }

        public int getGreen() {
            return green;
        }

        public int getBlue() {
            return blue;
        }

        public void setRed(int red) {
            this.red = red;
            this.red=Math.max(Math.min(this.red, 255), 0);
        }

        public void setGreen(int green) {
            this.green = green;
            this.green=Math.max(Math.min(this.green, 255), 0);
        }

        public void setBlue(int blue) {
            this.blue = blue;
            this.blue=Math.max(Math.min(this.blue, 255), 0);
        }

        public int[] getIntArray() {
            return new int[] { this.red, this.green, this.blue };
        }
    }
}