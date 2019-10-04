import java.awt.Color;
import java.util.Vector;
import java.util.Iterator;

public abstract class ColorConverter 
{ 
	static float[] temp=new float[3];
	protected int components;
	public ColorConverter() { components=3; }
	public final int getComponents() { return components; }
	float clip(float x) {
		return (x>=1?1:(x<=0?0:x));
	}
	float clip(int x) {
		return (x>=255?255:(x<=0?0:x));
	}
	public final float[] getColor(int c,float[] out) 
	{
		int c1=(c>>16)&0xff;
		int c2=(c>>8)&0xff;
		int c3=c&0xff;
		return getColor(c1, c2, c3,out);
	}
	public int getColor(int c) {
		int c1=(c>>16)&0xff;
		int c2=(c>>8)&0xff;
		int c3=c&0xff;
		return getColor(c1, c2, c3);
	}
	public int getColor(int c1, int c2, int c3) {
		getColor(c1,c2,c3,temp);
		return (Math.round(temp[0]*255)<<16) | (Math.round(temp[1]*255)<<8) | (Math.round(temp[2]*255));
	}
	public abstract float[] getColor(int c1, int c2, int c3, float[] out);
	public abstract float[] getColor(float c1, float c2, float c3, float[] out);


	public static class Compose extends ColorConverter {
		Vector v;
		public Compose() {
			super();
			v=new Vector();
		}
		public Compose(ColorConverter cc1, ColorConverter cc2) {
			super();
			v=new Vector();
			v.add(cc1);
			v.add(cc2);
		}
		
		public void add(ColorConverter cc) { v.add(cc); }
		
		public final float[] getColor(int c1,int c2,int c3, float[] out) { return getColor(c1/255.f,c2/255.f,c3/255.f,out); }
		
		public final float[] getColor(float c1, float c2, float c3, float[] out) {
			if(out==null)
				out=new float[3];
			out[0]=c1; out[1]=c2; out[2]=c3;
			for(Iterator it=v.iterator(); it.hasNext();)
				((ColorConverter)it.next()).getColor(out[0],out[1],out[2],out);
			return out;
		}
	}
	
	//use null instead
	/*
	public static class PassThrough extends ColorConverter {
		public final float[] getColor(int r, int g, int b, float[] out) {
			if(out==null)
				out=new float[3];
			out[0]=r/255.f;
			out[1]=g/255.f;
			out[2]=b/255.f;
			return out;
		}
		public final float[] getColor(float r, float g, float b, float[] out) {
			if(out==null)
				out=new float[]{r,g,b};
			else {
				out[0]=r;
				out[1]=g;
				out[2]=b;
			}
			return out;
		}
	}
	*/
	
	public static class RGBtoHSB extends ColorConverter {
		public final float[] getColor(int r, int g, int b, float[] hsb) {
			return Color.RGBtoHSB(r, g, b, hsb);
		}
		public final float[] getColor(float r, float g, float b, float[] hsb) {
			return Color.RGBtoHSB((int)(r*255), (int)(g*255), (int)(b*255), hsb);
		}
	}
	public static class HSBtoRGB extends ColorConverter {
		public final float[] getColor(int c1,int c2,int c3, float[] out) { return getColor(c1/255.f,c2/255.f,c3/255.f,out); }
		public final float[] getColor(float h, float s, float b, float[] rgb) {
			if(rgb==null)
				rgb=new float[3];
			int res=Color.HSBtoRGB(h, s, b);
			rgb[0]=((res>>16)&0xff)/255.f;
			rgb[1]=((res>>8)&0xff)/255.f;
			rgb[2]=(res&0xff)/255.f;
			return rgb;
		}			
	}
	public static class RGBtoRotatedHSB extends ColorConverter {
		final static float rotation=.2f;
		public final float[] getColor(int r, int g, int b, float[] hsb) {
			hsb=Color.RGBtoHSB(r, g, b, hsb);
			hsb[0]+=rotation;
			if(hsb[0]>1)
				hsb[0]-=1;
			return hsb;
		}
		public final float[] getColor(float r, float g, float b, float[] hsb) {
			hsb=Color.RGBtoHSB((int)(r*255), (int)(g*255), (int)(b*255), hsb);
			hsb[0]+=rotation;
			if(hsb[0]>1)
				hsb[0]-=1;
			return hsb;
		}
	}
	
	//these formulas are from http://en.wikipedia.org/wiki/YCbCr
	//The Y channel on the AIBO is scaled to [16-235] (inclusive)
	// so it's actually YCbCr, not YUV -- thus these formulas are for YCbCr, not YUV
	//I think some values might be off by 1 -- it's confusing whether to scale by 255 or 256, can change rounding direction
	public static class RGBtoUVY extends ColorConverter {
		public final float[] getColor(int R,int G,int B, float[] out) {
			if(out==null)
				out=new float[3];
			out[2] = clip(((66*R +  129*G +  25*B)>>8)+16) / 255.f;
			out[0] = clip(((-38*R -   74*G + 112*B)>>8)+128) / 255.f;
			out[1] = clip(((112*R -   94*G -  18*B)>>8)+128) / 255.f;
			return out;
		}
		public final float[] getColor(float R,float G,float B, float[] out) {
			if(out==null)
				out=new float[3];
			out[2] = clip( .2567891f*R + .5041289f*G + .0979062f*B + .0625f );
			out[0] = clip( -.1482227f*R -.2909922f*G + .4392148f*B + .5f );
			out[1] = clip( .4392148f*R -.3677891f*G  -.0714258f*B + .5f );
			return out;
		}
	}
	//inverted from http://en.wikipedia.org/wiki/YCbCr
	public static class YUVtoRGB extends ColorConverter {
		//final static ColorSpace cs=new ICC_ColorSpace(ICC_Profile.getInstance(ColorSpace.CS_PYCC));
		public final float[] getColor(int y,int u,int v, float[] out) {
			if(out==null)
				out=new float[3];
			out[0]=clip( ((298*y + 0*u + 409*v) >> 8) - 223 )/255.f;
			out[1]=clip( ((298*y - 100*u - 208*v) >> 8) + 136)/255.f;
			out[2]=clip( ((298*y + 516*u + 0*v) >> 8) - 227)/255.f;
			return out;
		}
		public final float[] getColor(float y,float u,float v, float[] out) {
			if(out==null)
				out=new float[3];
			out[0]=clip( 1.1644f*y + 0*u + 1.5960f*v - 0.8708f );
			out[1]=clip( 1.1644f*y - 0.3918f*u - 0.8130f*v + 0.5296f );
			out[2]=clip( 1.1644f*y + 2.0172f*u + 0*v - 1.0814f );
			return out;
		}
	}
		
	
	// this isn't the "real" conversion, nor a quick approx, but
	// it will get as close as possible to an exact undo of the YUV2RGB conversion
	// we use in VisionListener, so images saved from there will be
	// loaded without distorting color information (too much)
	public static class VisionListenerRGBtoYUV extends ColorConverter {
		public final float[] getColor(int r,int g,int b, float[] out) {
			if(out==null)
				out=new float[3];
			out[0]=clip( ( 8*r + 16*g  +  3*b)/(27.f * 255.f) );
			out[1]=clip( (-4*r  -  8*g  + 12*b + 3456)/(27.f * 255.f) ); 
			out[2]=clip( (19*r - 16*g  -   3*b + 3456)/(27.f * 255.f) );
			//if (y<0) y=0; if (u<0) u=0; if (v<0) v=0;
			//if (y>255) y=255; if (u>255) u=255; if (v>255) v=255;
			return out;
		}
		public final float[] getColor(float r,float g,float b, float[] out) { return getColor((int)(r*255),(int)(g*255),(int)(b*255),out); }
	}

	// this isn't the "real" conversion, but a quick approx.
	// it's the same that's used in tekkotsumon's VisionListener
	public static class YUVtoVisionListenerRGB extends ColorConverter {
		public final float[] getColor(int y,int u,int v, float[] out) {
			if(out==null)
				out=new float[3];
			u=u*2-256;
			v=v-128;
			int b=y+u;
			int r=y+v;
			v=v>>1;
			u=(u>>2)-(u>>4);
			int g=y-u-v;
			if (r<0) r=0; if (g<0) g=0; if (b<0) b=0;
			if (r>255) r=255; if (g>255) g=255; if (b>255) b=255;
			out[0]=r/255.f; out[1]=g/255.f; out[2]=b/255.f;
			return out;
		}
		public final float[] getColor(float y,float u,float v, float[] out) { return getColor((int)(y*255),(int)(u*255),(int)(v*255),out); }
	}
	
	public static class RGBtoxy_ extends ColorConverter {
		final static ColorConverter.RGBtoXYZ toXYZ=new ColorConverter.RGBtoXYZ();
		public final float[] getColor(int c1, int c2, int c3, float[] out) {
			if(out==null)
				out=new float[3];
			toXYZ.getColor(c1,c2,c3,out);
			float s=out[0]+out[1]+out[2];
			out[0]=clip(out[0]/s);
			out[1]=clip(out[1]/s);
			return out;
		}
		public final float[] getColor(float c1, float c2, float c3, float[] out) {
			if(out==null)
				out=new float[3];
			toXYZ.getColor(c1,c2,c3,out);
			float s=out[0]+out[1]+out[2];
			out[0]=clip(out[0]/s);
			out[1]=clip(out[1]/s);
			return out;
		}
	}
	
	public static class RGBtorg_ extends ColorConverter {
		final static ColorConverter.RGBtoXYZ toXYZ=new ColorConverter.RGBtoXYZ();
		public final float[] getColor(int c1, int c2, int c3, float[] out) {
			if(out==null)
				out=new float[3];
			float s=c1+c2+c3;
			out[0]=clip(c1/s);
			out[1]=clip(c2/s);
			return out;
		}
		public final float[] getColor(float c1, float c2, float c3, float[] out) {
			if(out==null)
				out=new float[3];
			float s=c1+c2+c3;
			out[0]=clip(c1/s);
			out[1]=clip(c2/s);
			return out;
		}
	}
	
	public static class RGBtoabL extends ColorConverter {
		final static ColorConverter.RGBtoXYZ toXYZ=new ColorConverter.RGBtoXYZ();
		final static ColorConverter.XYZtoabL toLab=new ColorConverter.XYZtoabL();
		public final float[] getColor(int c1, int c2, int c3, float[] out) {
			if(out==null)
				out=new float[3];
			toXYZ.getColor(c1,c2,c3,out);
			toLab.getColor(out[0],out[1],out[2],out);
			out[0]=clip( out[0]+.5f );
			out[1]=clip( out[1]+.5f );
			return out;
		}
		public final float[] getColor(float c1, float c2, float c3, float[] out) {
			if(out==null)
				out=new float[3];
			toXYZ.getColor(c1,c2,c3,out);
			toLab.getColor(out[0],out[1],out[2],out);
			out[0]=clip( out[0]+.5f );
			out[1]=clip( out[1]+.5f );
			return out;
		}
	}
	
	
	public static class RGBtoXYZ extends ColorConverter {
		final static float a=0.055f;
		final static float g=2.4f;
		static final int lookupRes=1024;
		static float[] powLookup=null;
		public RGBtoXYZ() {
			super();
			if(powLookup==null) {
				powLookup=new float[lookupRes];				
				for(int i=0; i<lookupRes; i++) {
					float K=i/(float)(lookupRes-1);
					if(K>0.04045f) {
						powLookup[i]=(float)Math.pow((K+a)/(1+a),g);
					} else {
						powLookup[i]=K/12.92f;
					}
				}
			}
		}
		final static float f(float K) {
			return powLookup[(int)(K*(lookupRes-1))];
		}
		public final float[] getColor(int c1, int c2, int c3, float[] out) {
			return getColor(c1/255.f,c2/255.f,c3/255.f,out);
		}
		public final float[] getColor(float c1, float c2, float c3, float[] out) {
			if(out==null)
				out=new float[3];
			float r=f(c1);
			float g=f(c2);
			float b=f(c3);
			out[0]=0.412424f*r + 0.357579f*g + 0.180464f*b;
			out[1]=0.212656f*r + 0.715158f*g + 0.072186f*b;
			out[2]=0.019332f*r + 0.119193f*g + 0.950444f*b;
			return out;
		}
	}
	
	public static class XYZtoabL extends ColorConverter {
		static final float Xn=0.31382f;
		static final float Yn=0.33100f;
		static final float Zn=(1-Xn-Yn);
		static final int lookupRes=1024;
		static float[] powLookup=null;
		public XYZtoabL() {
			super();
			if(powLookup==null) {
				powLookup=new float[lookupRes];				
				for(int i=0; i<lookupRes; i++)
					powLookup[i]=(float)Math.pow(4*i/(float)(lookupRes-1),1.0/3.0);
			}
		}
		final static float f(float t) {
			if(t>0.008856f) {
				return powLookup[(int)(t/4*(lookupRes-1))];
			} else {
				return 7.787f*t + 16.f/116.f;
			}
		}
		public final float[] getColor(int c1, int c2, int c3, float[] out) {
			return getColor(c1/255.f,c2/255.f,c3/255.f,out);
		}
		public final float[] getColor(float c1, float c2, float c3, float[] out) {
			if(out==null)
				out=new float[3];
			out[2]=116.f/256.f*f(c2/Yn)-16/156.f;
			out[0]=500.f/256.f*(f(c1/Xn)-f(c2/Yn));
			out[1]=200.f/256.f*(f(c2/Yn)-f(c3/Zn));
			return out;
		}
	}
	
	public static class YUVtoOther extends ColorConverter {
		ColorConverter cc;
		final static ColorConverter.YUVtoRGB toRGB=new ColorConverter.YUVtoRGB();
		public YUVtoOther(ColorConverter cc) {
			super();
			this.cc=cc;
		}
		public final float[] getColor(int c1, int c2, int c3, float[] out) {
			if(out==null)
				out=new float[3];
			toRGB.getColor(c1,c2,c3,out);
			return cc.getColor(out[0],out[1],out[2],out);
		}
		public final float[] getColor(float c1, float c2, float c3, float[] out) {
			if(out==null)
				out=new float[3];
			toRGB.getColor(c1,c2,c3,out);
			return cc.getColor(out[0],out[1],out[2],out);
		}
	}
	
	public static class YUVtoUVY extends ColorConverter {
		public final float[] getColor(int c1, int c2, int c3, float[] out) {
			if(out==null)
				out=new float[3];
			out[0]=c2/255.f;
			out[1]=c3/255.f;
			out[2]=c1/255.f;
			return out;
		}
		public final float[] getColor(float c1, float c2, float c3, float[] out) {
			if(out==null)
				out=new float[3];
			out[0]=c2;
			out[1]=c3;
			out[2]=c1;
			return out;
		}
	}
	
	public static class Scale extends ColorConverter {
		ColorConverter cc;
		float s;
		public Scale(ColorConverter cc, float s) {
			super();
			this.cc=cc;
			this.s=s;
		}
		public final float[] getColor(int c1, int c2, int c3, float[] out) {
			float[] o=cc.getColor(c1,c2,c3,out);
			o[0]=clip(o[0]*s);
			o[1]=clip(o[1]*s);;
			o[2]=clip(o[2]*s);;
			return o;
		}
		public final float[] getColor(float c1, float c2, float c3, float[] out) {
			float[] o=cc.getColor(c1,c2,c3,out);
			o[0]=clip(o[0]*s);;
			o[1]=clip(o[1]*s);;
			o[2]=clip(o[2]*s);;
			return o;
		}
	}
}
