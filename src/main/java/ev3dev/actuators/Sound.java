package ev3dev.actuators;

import ev3dev.hardware.EV3DevDevice;
import ev3dev.hardware.EV3DevPlatform;
import ev3dev.utils.Shell;
import lejos.utility.Delay;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.sound.sampled.*;
import java.io.File;
import java.io.IOException;
import java.util.Objects;

/**
 * Class that provides access methods for the local audio device
 *
 * The class is implemented as Singleton.
 *
 * Note: Only tested with EV3Brick
 *
 * @author Juan Antonio Breña Moral
 *
 */
public class Sound extends EV3DevDevice {

    private static final Logger LOGGER = LoggerFactory.getLogger(Sound.class);

    private static final String EV3_PHYSICAL_SOUND_PATH = "/sys/devices/platform/snd-legoev3";
    public  static final String EV3DEV_SOUND_KEY = "EV3DEV_SOUND_KEY";
    private static String EV3_SOUND_PATH;

    private static final String CMD_BEEP = "beep";
    public  static final String VOLUME = "volume";

    private static String VOLUME_PATH;
    private final static  String DISABLED_FEATURE_MESSAGE = "This feature is disabled for this platform.";

    /**
     *  The sample rate - 44,100 Hz for CD quality audio.
     */
    public static final int SAMPLE_RATE = 44100;

    private static final int BYTES_PER_SAMPLE = 2;                // 16-bit audio
    private static final int BITS_PER_SAMPLE = 16;                // 16-bit audio
    private static final double MAX_16_BIT = Short.MAX_VALUE;     // 32,767
    private static final int SAMPLE_BUFFER_SIZE = 4096;


    private static SourceDataLine line;   // to play the sound
    private static byte[] buffer;         // our internal buffer
    private static int bufferSize = 0;    // number of samples currently in internal buffer

    private int volume = 0;

    private static Sound instance;

    /**
     * Return a Instance of Sound.
     *
     * @return A Sound instance
     */
    public static Sound getInstance() {

        LOGGER.info("Providing a Sound instance");

        if (Objects.isNull(instance)) {
            instance = new Sound();
        }
        return instance;
    }

    // Prevent duplicate objects
    private Sound() {

        LOGGER.info("Creating a instance of Sound");

        EV3_SOUND_PATH  = Objects.nonNull(System.getProperty(EV3DEV_SOUND_KEY)) ? System.getProperty(EV3DEV_SOUND_KEY) : EV3_PHYSICAL_SOUND_PATH;
        VOLUME_PATH = EV3_SOUND_PATH + "/" + VOLUME;

        init();
    }

    // open up an audio stream
    private void init() {
        try {
            // 44,100 samples per second, 16-bit audio, mono, signed PCM, little Endian
            AudioFormat format = new AudioFormat((float) SAMPLE_RATE, BITS_PER_SAMPLE, 1, true, false);
            DataLine.Info info = new DataLine.Info(SourceDataLine.class, format);

            line = (SourceDataLine) AudioSystem.getLine(info);
            line.open(format, SAMPLE_BUFFER_SIZE * BYTES_PER_SAMPLE);

            // the internal buffer is a fraction of the actual buffer size, this choice is arbitrary
            // it gets divided because we can't expect the buffered data to line up exactly with when
            // the sound card decides to push out its samples.
            buffer = new byte[SAMPLE_BUFFER_SIZE * BYTES_PER_SAMPLE/3];
        }
        catch (LineUnavailableException e) {
            System.out.println(e.getMessage());
        }

        // no sound gets made before this call
        line.start();
    }

    /**
     * Beeps once.
     */
    public void beep() {
        if(this.getPlatform().equals(EV3DevPlatform.EV3BRICK)){
            LOGGER.debug(CMD_BEEP);
            Shell.execute(CMD_BEEP);
            Delay.msDelay(100);
        } else {
            LOGGER.warn(DISABLED_FEATURE_MESSAGE);
        }
    }

    /**
     * Beeps twice.
     */
    public void twoBeeps() {
        if(this.getPlatform().equals(EV3DevPlatform.EV3BRICK)){
            beep();
            beep();
        } else {
            LOGGER.debug(DISABLED_FEATURE_MESSAGE);
        }
    }

    /**
     * Plays a tone, given its frequency and duration. 
     * @param frequency The frequency of the tone in Hertz (Hz).
     * @param duration The duration of the tone, in milliseconds.
     * @param volume The volume of the playback 100 corresponds to 100%
     */
    public void playTone(final int frequency, final int duration, final int volume) {
        if(this.getPlatform().equals(EV3DevPlatform.EV3BRICK)){
            this.setVolume(volume);
    	    this.playTone(frequency, duration);
        } else {
            LOGGER.debug(DISABLED_FEATURE_MESSAGE);
        }
    }

    public static double[] tone(double hz, double duration) {
        int n = (int) (SAMPLE_RATE * duration);
        double[] a = new double[n+1];
        for (int i = 0; i <= n; i++) {
            a[i] = Math.sin(2 * Math.PI * i * hz / SAMPLE_RATE);
        }
        return a;
    }

    /**
     * Plays a tone, given its frequency and duration. 
     * @param frequency The frequency of the tone in Hertz (Hz).
     * @param duration The duration of the tone, in milliseconds.
     */
    public void playTone(final int frequency, final int duration) {
        if(this.getPlatform().equals(EV3DevPlatform.EV3BRICK)) {

            // create the array
            double[] a = tone(frequency, duration);

            // play it using standard audio
            play(a);

        } else {
            LOGGER.debug(DISABLED_FEATURE_MESSAGE);
        }
    }

    /**
     * Writes one sample (between -1.0 and +1.0) to standard audio.
     * If the sample is outside the range, it will be clipped.
     *
     * @param  sample the sample to play
     * @throws IllegalArgumentException if the sample is {@code Double.NaN}
     */
    public static void play(double sample) {

        // clip if outside [-1, +1]
        if (Double.isNaN(sample)) throw new IllegalArgumentException("sample is NaN");
        if (sample < -1.0) sample = -1.0;
        if (sample > +1.0) sample = +1.0;

        // convert to bytes
        short s = (short) (MAX_16_BIT * sample);
        buffer[bufferSize++] = (byte) s;
        buffer[bufferSize++] = (byte) (s >> 8);   // little Endian

        // send to sound card if buffer is full
        if (bufferSize >= buffer.length) {
            line.write(buffer, 0, buffer.length);
            bufferSize = 0;
        }
    }

    /**
     * Writes the array of samples (between -1.0 and +1.0) to standard audio.
     * If a sample is outside the range, it will be clipped.
     *
     * @param  samples the array of samples to play
     * @throws IllegalArgumentException if any sample is {@code Double.NaN}
     * @throws IllegalArgumentException if {@code samples} is {@code null}
     */
    public static void play(double[] samples) {
        if (samples == null) throw new IllegalArgumentException("argument to play() is null");
        for (int i = 0; i < samples.length; i++) {
            play(samples[i]);
        }
    }

    /**
     * Play a wav file. Must be mono, from 8kHz to 48kHz, and 8-bit or 16-bit.
     * @param file the 8-bit or 16-bit PWM (WAV) sample file
     * @param volume the volume percentage 0 - 100
     */
    public void playSample(final File file, final int volume) {
        this.setVolume(volume);
        this.playSample(file);
    }


    /**
     * Play a wav file. Must be mono, from 8kHz to 48kHz, and 8-bit or 16-bit.
     * @param file the 8-bit or 16-bit PWM (WAV) sample file
     */
    public void playSample(final File file) {
        try (AudioInputStream audioIn = AudioSystem.getAudioInputStream(file.toURI().toURL())) {

            Clip clip = AudioSystem.getClip();
            clip.open(audioIn);
            clip.start();
            Delay.usDelay(clip.getMicrosecondLength());;

        } catch (IOException | LineUnavailableException | UnsupportedAudioFileException e) {
            LOGGER.error(e.getLocalizedMessage(), e);
            throw new RuntimeException(e);
        }
    }

    /**
     * Set the master volume level
     * @param volume 0-100
     */
    public void setVolume(final int volume) {
        this.volume = volume;
        final String cmdVolume = "amixer set PCM,0 " + volume + "%";
        Shell.execute(cmdVolume);
    }

    /**
     * Get the current master volume level
     * @return the current master volume 0-100
     */
    public int getVolume() {
        return this.volume;
    }

    /**
     * Closes standard audio.
     */
    public static void close() {
        line.drain();
        line.stop();
    }

}
