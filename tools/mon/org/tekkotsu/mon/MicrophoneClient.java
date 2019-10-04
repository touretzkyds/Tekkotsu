package org.tekkotsu.mon;

import java.io.InputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Streams audio from the AIBO's microphones.
 *
 * Every packet of audio received contains information about sample rate, bit
 * depth and the number of channels. The parameters are on the mercy of the
 * server and can change during streaming.
 *
 * @author Alexander Klyubin
 */
public class MicrophoneClient extends TCPListener {
  private final List audioListeners = new ArrayList();
  private static final AudioListener[] EMPTY_AUDIO_LISTENER_ARRAY =
    new AudioListener[0];
  private AudioListener[] cachedAudioListeners = EMPTY_AUDIO_LISTENER_ARRAY;

  /**
   * Notifies of arrival of streamed audio packets.
   */
  public interface AudioListener {
    /**
     * Notifies that an audio packet has been received.
     * @param samples samples (PCM-encoded).
     * @param sampleRate sample rate.
     * @param bitsPerSample bits per sample (usually <code>8</code> or
     *        <code>16</code>).
     * @param stereo <code>true</code> if <code>samples</code> contain two
     *        channels (stereo), <code>false</code> if <code>samples</code>
     *        contain only one channel (mono).
     */
    void onSamplesReceived(
      byte[] samples,
      int sampleRate,
      int bitsPerSample,
      boolean stereo);
  }
	
  /**
   * Adds an audio listener.
   *
   * @param listener listener to add.
   */
  public void addAudioListener(AudioListener listener) {
    audioListeners.add(listener);
    cachedAudioListeners =
      (AudioListener[]) audioListeners.toArray(EMPTY_AUDIO_LISTENER_ARRAY);
	needConnection(); 
  }
  
  /**
   * Removes an audio listener.
   *
   * @param listener listener to remove.
   */
  public void removeAudioListener(AudioListener listener) {
    while (audioListeners.remove(listener)) {}
    cachedAudioListeners =
      (AudioListener[]) audioListeners.toArray(EMPTY_AUDIO_LISTENER_ARRAY);
  }
  
  /**
   * Notifies listeners that an audio packet has been received from the
   * microphone.
   *
   * @param samples samples (PCM-encoded).
   * @param sampleRate sample rate.
   * @param bitsPerSample bits per sample (usually <code>8</code> or
   *        <code>16</code>).
   * @param stereo <code>true</code> if <code>samples</code> contain two
   *        channels (stereo), <code>false</code> if <code>samples</code>
   *        contain only one channel (mono).
   */
  protected void fireSamplesReceived(
    byte[] samples,
    int sampleRate,
    int bitsPerSample,
    boolean stereo) {
      
    for (int i = 0, len = cachedAudioListeners.length; i < len; i++) {
      cachedAudioListeners[i].onSamplesReceived(
        samples,
        sampleRate,
        bitsPerSample,
        stereo);
    }
  }
  
  /**
   * Handles a connection.
   *
   * @param socket socket connected to the microphone server. 
   */
  protected void connected(Socket socket) {
    InputStream in  = null; 
		try {
			in = socket.getInputStream();
      fireConnected();
			while (true) {
        if (!_isConnected) {
					break;
        }
				consumeFrame(in);
				if (!_isConnected) {
					break;
        }
			}
		} catch (Exception e) {
      e.printStackTrace();
    } finally {
      if (in != null) {
        try {
          in.close();
        } catch (IOException e) {
          e.printStackTrace();
        }
        in = null;
      }
      fireDisconnected();
    }
		
		try { socket.close(); } catch (IOException e) {}
	}
  
  /**
   * Converts a signed <code>short</code> into unsigned form.
   * 
   * @param value value to convert.
   *
   * @return unsigned version of the <code>value</code>.
   */
  private static int unsignedShort(short value) {
    return ((value >= 0) ? value : 65536 + value);
  }
  
  /**
   * Consumes an audio frame from the stream.
   * 
   * @param in input stream to consume the frame from.
   * @throws IOException if an I/O exception occurs.
   */
  protected void consumeFrame(InputStream in) throws IOException {
    // HEADER (4 bytes)
    //   unsigned short: size
    //   unsigned short: type
    // DATA (size)
    final int size = unsignedShort(readShort(in));
    final int type = unsignedShort(readShort(in));
    if (type == 0) {
      // PCM frame
      // HEADER (4 bytes)
      //   unsigned short: sampleRate
      //   byte:           bitsPerSample
      //   byte:           stereo
      // PCM DATA (size - 4)
      final int sampleRate = unsignedShort(readShort(in));
      final byte bitsPerSample = readByte(in);
      final boolean stereo = (readByte(in) == 1);
      final byte[] frame = readBytes(in, size - 4);
      fireSamplesReceived(frame, sampleRate, bitsPerSample, stereo);
    } else {
      System.err.println("Unknown microphone frame type: " + type
        + ", size = " + size);
      // Skip the frame
      readBytes(in, size);
    }
  }
  
  /**
   * Constructs a new <code>MicrophoneClient</code> instance.
   */
  public MicrophoneClient() {}
}
