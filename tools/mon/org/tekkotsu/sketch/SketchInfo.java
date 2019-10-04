package org.tekkotsu.sketch;

import org.tekkotsu.mon.TCPVisionListener;
import org.tekkotsu.mon.ColorConverter;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import javax.swing.Icon;
import javax.swing.ImageIcon;

import java.util.Hashtable;
import java.awt.image.IndexColorModel;
import java.awt.Graphics;

// stores info for a Sketch, to use as UserObject for DefaultMutableTreeNode
public class SketchInfo extends SketchOrShapeInfo {
    static Icon bool_icon = new ImageIcon(icon_path+"sketchbool.png");
    static Icon inverted_bool_icon = new ImageIcon(icon_path+"sketchboolinv.png");;

    static Icon uchar_icon = new ImageIcon(icon_path+"sketchuchar.png");
    static Icon inverted_uchar_icon = new ImageIcon(icon_path+"sketchucharinv.png");

    static Icon int_icon = new ImageIcon(icon_path+"sketchint.png");
    static Icon inverted_int_icon = new ImageIcon(icon_path+"sketchintinv.png");

    static Icon usint_icon = new ImageIcon(icon_path+"sketchusint.png");
    static Icon inverted_usint_icon = new ImageIcon(icon_path+"sketchusintinv.png");

    static Icon yuv_icon = new ImageIcon(icon_path+"sketchyuv.png");
    static Icon inverted_yuv_icon = new ImageIcon(icon_path+"sketchyuvinv.png");

    static Icon gray_icon = new ImageIcon(icon_path+"sketchgray.png");
    static Icon inverted_gray_icon = new ImageIcon(icon_path+"sketchgrayinv.png");


    public static final int SKETCH_BOOL_TYPE  = 1;
    public static final int SKETCH_UCHAR_TYPE  = 2;
    public static final int SKETCH_USINT_TYPE   = 3;
    public static final int SKETCH_UINT_TYPE   = 4;
    public static final int SKETCH_FLOAT_TYPE = 5;
    public static final int SKETCH_YUV_TYPE = 6;
    

    public static final int COLORMAP_SEG_TYPE = 0;
    public static final int COLORMAP_GRAY_TYPE = 1;
    public static final int COLORMAP_JET_TYPE = 2;
    public static final int COLORMAP_JET_SCALED_TYPE = 3;

    boolean imageLoaded;
    BufferedImage img;
    int sketchType;
    int pixelSize;
    int grayColor = Color.GRAY.getRGB();   // zero pixels are shown as gray in sketch bool
    int colormap;
    int width;
    int height;


    byte jetred[]; 
    byte jetgreen[];
    byte jetblue[];

    float pixelMax, pixelMin;


    public SketchInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color, 
                      int _colormap, int _sketchType, int _width, int _height) {
        super(_gui, _id, _parentId, _name, _color);
        colormap = _colormap;
        width = _width;
        height = _height;
        imageLoaded = false;
        img = null;
        sketchType = _sketchType;
        if ( sketchType == SKETCH_BOOL_TYPE || sketchType == SKETCH_UCHAR_TYPE )
            pixelSize = 1;
        else if ( sketchType == SKETCH_USINT_TYPE )
            pixelSize = 2;
        else if ( sketchType == SKETCH_YUV_TYPE )
            pixelSize = 3;
        else if ( sketchType == SKETCH_UINT_TYPE || sketchType == SKETCH_FLOAT_TYPE )
            pixelSize=4;
        else // unrecognized type
            pixelSize = 1;

        makeJetMap();
    }
    
    public Icon getIcon() { 
        if ( !inverted ) {
            if (sketchType == SKETCH_BOOL_TYPE) return bool_icon;
            else if (colormap == COLORMAP_GRAY_TYPE) return gray_icon;
            else if (sketchType == SKETCH_UCHAR_TYPE)  return uchar_icon;
            else if (sketchType == SKETCH_USINT_TYPE)  return usint_icon;
            else if (sketchType == SKETCH_YUV_TYPE)  return yuv_icon;
            else return int_icon;
        }
        else {
            if (sketchType == SKETCH_BOOL_TYPE) return inverted_bool_icon;
            else if (colormap == COLORMAP_GRAY_TYPE) return inverted_gray_icon;
            else if (sketchType == SKETCH_UCHAR_TYPE)  return inverted_uchar_icon;
            else if (sketchType == SKETCH_USINT_TYPE)  return inverted_usint_icon;
            else if (sketchType == SKETCH_YUV_TYPE)  return inverted_yuv_icon;
            else return inverted_int_icon;
        }
    }

    public boolean isImageLoaded() { return imageLoaded; }

    public BufferedImage getImage() { return img; }

    public int getSketchType() { return sketchType; }

    public void unloadImage() {
        imageLoaded = false;
        img = null;
    }

    // Copy image from TCPVisionListener's buffer to our own buffer, and find min max pixel values.
    public void copyImage(BufferedImage vis_img) {
        if (vis_img == null) {
            System.err.println("Tried to copy a null image");
            return;
        }

        if ( pixelSize == 1 )
            img = new BufferedImage(vis_img.getWidth(), vis_img.getHeight(),vis_img.getType(),
                                    ((IndexColorModel) vis_img.getColorModel()));
        else
            img = new BufferedImage(vis_img.getWidth(), vis_img.getHeight(), BufferedImage.TYPE_4BYTE_ABGR);
        img.getRaster().setRect(vis_img.getRaster());
        pixelMax = -Float.MAX_VALUE;
        pixelMin = Float.MAX_VALUE;
        int len = vis_img.getWidth()*vis_img.getHeight();
        for (int i=0; i<len; i++) {
            int x = i % vis_img.getWidth();
            int y = i / vis_img.getWidth();
            int pixel = vis_img.getRaster().getSample(x,y,0);
            if ( pixelSize > 1 ) {
                pixel |= (vis_img.getRaster().getSample(x,y,1) << 8);
                if ( pixelSize > 2 ) {
                    pixel |= (vis_img.getRaster().getSample(x,y,2) << 16);
                    if ( pixelSize > 3 )
                        pixel |= (vis_img.getRaster().getSample(x,y,3) << 24);
                }
            }
            float floatpixel = ( sketchType != SKETCH_FLOAT_TYPE ) ? (float)pixel : Float.intBitsToFloat(pixel);
            if (floatpixel > pixelMax)
                pixelMax = floatpixel;
            else if (floatpixel < pixelMin)
                pixelMin = floatpixel;
        }
        System.out.println("Image type="+sketchType+", len="+len+", colormap="+colormap+", pixelSize="+pixelSize+
                           ", pixel range=("+pixelMin+" to "+pixelMax+")");
        if (pixelMax == pixelMin) ++pixelMax;
        imageLoaded = true;
    }


    public void renderToArrays(int r[], int g[], int b[], int counts[]) {
        if (img == null)
            return;
        else if ( sketchType == SKETCH_UCHAR_TYPE && colormap == COLORMAP_SEG_TYPE || sketchType == SKETCH_BOOL_TYPE )
            renderToArrays_seg(r, g, b, counts);
        else if ( sketchType == SKETCH_UCHAR_TYPE && colormap == COLORMAP_GRAY_TYPE )
            renderToArrays_gray(r, g, b, counts);
        else if ( sketchType == SKETCH_YUV_TYPE )
            renderToArrays_yuv(r, g, b, counts);
        else if (colormap == COLORMAP_JET_TYPE || colormap == COLORMAP_JET_SCALED_TYPE ||
                 sketchType == SKETCH_USINT_TYPE || sketchType == SKETCH_UINT_TYPE || sketchType == SKETCH_FLOAT_TYPE)
            renderToArrays_jet(r, g, b, counts);
        else
            System.out.println("Invalid combination: sketch type "+sketchType+"  color map "+colormap);
    }

    public void renderToArrays_seg(int r[], int g[], int b[], int counts[]) {
        Color color = ColorConverter.convertColor(getColor());
        int curR = color.getRed();                // set up color info for sketch bool; automatically handles inverted flag
        int curG = color.getGreen();
        int curB = color.getBlue();
        int pos = 0;
        for (int y = 0; y < img.getHeight(); y++) {
            for (int x = 0; x < img.getWidth(); x++) {
                int cur = img.getRGB(x,y);
                if (cur == grayColor)
                    pos++;
                else {
                    if (sketchType != SKETCH_BOOL_TYPE) {
                        Color c = new Color(cur);
                        curR = c.getRed();
                        curG = c.getGreen();
                        curB = c.getBlue();
                        if (inverted) {
                            curR = 255 - curR;
                            curG = 255 - curG;
                            curB = 255 - curB;
                        }
                    }
                    r[pos] += curR;
                    g[pos] += curG;
                    b[pos] += curB;
                    counts[pos]++;
                    pos++;
                }}}}


    public void renderToArrays_gray(int r[], int g[], int b[], int counts[]) {
        int pos = 0;
        for (int y = 0; y < img.getHeight(); y++) {
            for (int x = 0; x < img.getWidth(); x++) {
                int cur = img.getRaster().getSample(x,y,0);
                if (inverted)
                    cur = 255 - cur;
                r[pos]+=cur;
                g[pos]+=cur;
                b[pos]+=cur;
                counts[pos]++;
                pos++;
            }}}


    public void renderToArrays_yuv(int r[], int g[], int b[], int counts[]) {
        int pos = 0;
        for (int y = 0; y < img.getHeight(); y++) {
            for (int x = 0; x < img.getWidth(); x++) {
                    int cur = TCPVisionListener.pixelYUV2RGB(img.getRaster().getSample(x,y,0),
                                                             img.getRaster().getSample(x,y,1),
                                                             img.getRaster().getSample(x,y,2));
                    int curR = (cur & 0xff0000) >> 16;
                    int curG = (cur & 0xff00) >> 8;
                    int curB = cur & 0xff; 
                    if ( inverted ) {
                        curR = 255 - curR;
                        curG = 255 - curG;
                        curB = 255 - curB;
                    }
                    r[pos] += curR;
                    g[pos] += curG;
                    b[pos] += curB;
                    counts[pos]++;
                    pos++;
            }}}

    public void renderToArrays_jet(int r[], int g[], int b[], int counts[]) {
        int pos = 0;
        for (int y = 0; y < img.getHeight(); y++) {
            for (int x = 0; x < img.getWidth(); x++) {
                int pixel = img.getRaster().getSample(x,y,0);
                if ( pixelSize > 1 ) {
                    pixel |= img.getRaster().getSample(x,y,1) << 8;
                    if ( pixelSize > 2 )
                        pixel |= img.getRaster().getSample(x,y,2) << 16 | img.getRaster().getSample(x,y,3) << 24;
                }
                float floatpixel = (sketchType != SKETCH_FLOAT_TYPE) ? pixel : Float.intBitsToFloat(pixel);
                int colorindex;
                if (colormap == COLORMAP_JET_SCALED_TYPE)
                    colorindex = (int)Math.ceil(255.0*(floatpixel - pixelMin)/(pixelMax - pixelMin));
                else if (floatpixel > 255.0)
                    colorindex = 255;
                else if (floatpixel < 0)
                    colorindex = 0;
                else
                    colorindex = (int)floatpixel;
                if (inverted)
                    colorindex = 255 - colorindex;
                r[pos] += 0x00FF & (int)jetred[colorindex];
                g[pos] += 0x00FF & (int)jetgreen[colorindex];
                b[pos] += 0x00FF & (int)jetblue[colorindex];
                counts[pos]++;
                pos++;
            }}}

    public void makeJetMap() {
        jetred = new byte[256];
        jetgreen = new byte[256];
        jetblue = new byte[256];

        for (int i=0;i<256; i++) {
            if (i < 32)
                jetgreen[i] = 0;
            else if (i < 96)
                jetgreen[i] = (byte) ((i-32)*4);
            else if (i <= 160)
                jetgreen[i] = (byte)255;
            else if (i < 224)
                jetgreen[i] = (byte) ((224-i)*4);
            else
                jetgreen[i] = 0;
        }

        for (int i=0; i<256; i++) {
            if (i < 192)
                jetblue[i] = jetgreen[i+64];
            else
                jetblue[i] = 0;
                
            if (i >= 64)
                jetred[i] = jetgreen[i-64];
            else
                jetred[i] = 0;
        }

        // manually set color 0 to background gray
        
        jetred[0] = (byte)Color.GRAY.getRed();
        jetgreen[0] = (byte)Color.GRAY.getGreen();
        jetblue[0] = (byte)Color.GRAY.getBlue();
    }
    
}
