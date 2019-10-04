package org.tekkotsu.mon;

import java.io.InputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;

/**
 * Streams audio to the AIBO's speaker.
 *
 * Each packet sent contains sample rate and bit depth information which is
 * used by the server to resample it to the correct format used by the AIBO
 * internally.
 *
 * @author Alexander Klyubin
 */
public class SpeakerClient extends TCPListener {
  /** Output stream for communication with the server. */
  OutputStream out = null;
  
  /**
   * Handles a connection.
   *
   * @param socket socket connected to the speaker server. 
   */
  protected void connected(Socket socket) {
    InputStream in  = null; 
		try {
      out = socket.getOutputStream();
			in = socket.getInputStream();
      fireConnected();
			while (true) {
        if (!_isConnected) {
					break;
        }
        if (in.read() == -1) {
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
      if (out != null) {
        try {
          out.close();
        } catch (IOException e) {
          e.printStackTrace();
        }
        out = null;
      }
      fireDisconnected();
    }
		
		try { socket.close(); } catch (IOException e) {}
	}
  
  /**
   * Sends a frame of audio to the speaker.
   * 
   * @param samples samples (PCM-encoded, 16-bit signed or 8-bit unsigned).
   * @param sampleRate sample rate (Hz).
   * @param bitsPerSample bits per sample (usually <code>8</code> or
   *        <code>16</code>).
   *
   * @throws IOException if an I/O exception occurs.
   */
  public void sendFrame(
    byte[] samples,
    int sampleRate,
    int bitsPerSample) throws IOException {
      
    if (samples.length > 65530) {
      throw new IllegalArgumentException(
        "Frame too long. Maximum length: 65530");
    }
    
    if ((!_isConnected) || (out == null)) {
      throw new IOException("Not connected");
    }
    
    // HEADER (4 bytes)
    //   unsigned short: size
    //   unsigned short: type
    // DATA (size)
    writeShort(out, (short) (samples.length + 4));
    writeShort(out, (short) 0); // PCM
    // PCM frame
    // HEADER (4 bytes)
    //   unsigned short: sampleRate
    //   byte:           bitsPerSample
    //   byte:           padding
    // PCM DATA (size - 4)
    writeShort(out, (short) sampleRate);
    writeByte(out, (byte) bitsPerSample);
    writeByte(out, (byte) 0);
    writeBytes(out, samples);
    out.flush();
  }
  
  /**
   * Constructs a new <code>SpeakerClient</code>.
   */
  public SpeakerClient() {}
}
