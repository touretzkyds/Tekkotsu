import java.awt.geom.*;
import java.awt.Color;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;
import java.io.File;
import java.io.FileWriter;
import java.awt.image.*;

public class ThresholdMap {
	ColorConverter convert;
	int yRes,uRes,vRes;
	short[][][] map; //short because no unsigned byte type :-P
	byte[] repR;
	byte[] repG;
	byte[] repB;
	TreeMap areas;
	
	// these defaults drop the highest and lowest y level, so these
	// y-values won't map to anything (too desaturated or washed out)
	final int MIN_Y=1; // minimum y level to be mapped
	final int MAX_Y=-1; // offset from y res -- maximum saturation to be mapped
	
	public ThresholdMap(ColorConverter convert, int yRes, int uRes, int vRes) {
		this.convert=convert;
		this.yRes=yRes;
		this.uRes=uRes;
		this.vRes=vRes;
		map=new short[yRes][uRes][vRes];
		repR=new byte[256];
		repG=new byte[256];
		repB=new byte[256];
		resetMap();
		areas=new TreeMap();
	}
	
	public Color getRepresentitiveColor(int i) {
		return new Color((repR[i]<<16)+(repG[i]<<8)+repB[i]);
	}
	
	public void resetMap() {
		for(int y=0; y<yRes; y++)
			for(int u=0; u<uRes; u++)
				for(int v=0; v<vRes; v++)
					map[y][u][v]=Short.MAX_VALUE;
	}

	public void changeSpectrum(ColorConverter convert) {
		this.convert=convert;
		for(Iterator it=areas.entrySet().iterator(); it.hasNext();) {
			Map.Entry entry=(Map.Entry)it.next();
			updateMap((Area)entry.getValue(),((Short)entry.getKey()).shortValue());
		}
	}
	
	public void updateMap(Area area, short index) {
		float[] xy=new float[convert.getComponents()];
		areas.put(new Short(index),area.clone());
		for(int y=MIN_Y; y<yRes+MAX_Y; y++) {
			float yc=(y+.5f)/yRes;
			for(int u=0; u<uRes; u++) {
				float uc=(u+.5f)/uRes;
				for(int v=0; v<vRes; v++) {
					
					if(map[y][u][v]>index) {
						//have to test if a new mapping was added
						float vc=(v+.5f)/vRes;
						convert.getColor(yc,uc,vc,xy);
						//ugly hack because points on the area border aren't normally considered hits, but need to be
						//boolean hit=area.contains(xy[0]*.999+.0005,xy[1]*.999+.0005);
						boolean hit=area.contains(xy[0],xy[1]);
						if(hit)
							map[y][u][v]=index;
					} else if(map[y][u][v]==index) {
						//have to test if current mapping was removed
						float vc=(v+.5f)/vRes;
						convert.getColor(yc,uc,vc,xy);
						//ugly hack because points on the area border aren't normally considered hits, but need to be
						//boolean hit=area.contains(xy[0]*.999+.0005,xy[1]*.999+.0005);
						boolean hit=area.contains(xy[0],xy[1]);
						if(!hit) {
							map[y][u][v]=Short.MAX_VALUE; //mapping went away
							// but now need to see if it "fell through" to another layer
							Iterator it=areas.entrySet().iterator();
							while(it.hasNext()) {
								Map.Entry entry=(Map.Entry)it.next();
								short k=((Short)entry.getKey()).shortValue();
								if(k<=index)
									continue;
								Area a=(Area)entry.getValue();
								if(a.contains(xy[0],xy[1])) {
									map[y][u][v]=k;
									break;
								}
							}
						}
					}
					//don't have to test if on lower-index layer (they would still override any changes to current index)
					
				} // main body, v loop
			} // u loop
		}// y loop
		float repyc=.6f;
		int repy=(int)(repyc*(yRes-1));
		float rc=0,gc=0,bc=0;
		int c=0;
		ColorConverter YUVtoRGB=new ColorConverter.YUVtoRGB();
		for(int u=0; u<uRes; u++) {
			float uc=u/(float)(uRes-1);
			for(int v=0; v<vRes; v++) {
				float vc=v/(float)(vRes-1);
				if(map[repy][u][v]==index) {
					YUVtoRGB.getColor(repyc,uc,vc,xy);
					rc+=xy[0];
					gc+=xy[1];
					bc+=xy[2];
					c++;
				}
			}
		}
		repR[index]=(byte)(255*rc/c);
		repG[index]=(byte)(255*gc/c);
		repB[index]=(byte)(255*bc/c);
	}
	
	public BufferedImage segment(float[] img, int width, int height) {
		IndexColorModel model=new IndexColorModel(8,256,repR,repG,repB,255);
		BufferedImage out = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_INDEXED,model);
		byte[] seg=new byte[img.length];
		int i=0;
		int j=0;
		while(i<img.length) {
			int y=(int)(clip(img[i++])*(yRes-1));
			int u=(int)(clip(img[i++])*(uRes-1));
			int v=(int)(clip(img[i++])*(vRes-1));
			seg[j++]=(byte)map[y][u][v];
		}
		out.getRaster().setDataElements(0,0,width,height,seg);//new DataBufferByte(seg,seg.length));
		return out;
	}
	
	float clip(float x) {
		return x<=0 ? 0 : (x>=1 ? 1 : x);
	}
	
	public void saveFile(File f) throws java.io.IOException {
		FileWriter save=new FileWriter(f);
		save.write("TMAP\nYUV'8\n" + yRes + " " + uRes + " " + vRes + "\n");
		for(int y=0; y<yRes; y++)
			for(int u=0; u<uRes; u++)
				for(int v=0; v<vRes; v++)
				save.write((byte)(map[y][u][v]+1)); //write out all of v at once
		save.close();
	}
}
