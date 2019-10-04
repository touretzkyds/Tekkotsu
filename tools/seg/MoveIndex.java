import java.util.*;
import java.io.*;

public class MoveIndex {
	public static void main(String[] args) {
		if(args.length<3) {
			System.out.println("Usage: java MoveIndex oldindex newindex infile [outfile]");
			System.out.println("       Pass just the base filename (no extension)");
			System.out.println("       Will read corresponding .tm and .col files and write updated files");
			System.out.println("       If outfile is not provided, results will be written back to infile");
			System.exit(2);
		}
		int oldi=Integer.parseInt(args[0]);
		int newi=Integer.parseInt(args[1]);
		String inf=args[2];
		String outf=args[2];
		if(args.length>3)
			outf=args[3];
		
    try {
			//Move index found in tm file
			{
				byte[] header=new byte[20];
				byte[] tmdata=new byte[65536];
				
				FileInputStream in=new FileInputStream(inf+".tm");
				in.read(header);
				in.read(tmdata);
					
				for (int j=0; j<tmdata.length; j++)
					if (tmdata[j]==oldi) tmdata[j]=(byte)newi;
					
				in.close();
					
				FileOutputStream out=new FileOutputStream(outf+".tm");
				out.write(header);
				out.write(tmdata);
				out.close();
			}
			
			//Move index found in col file
			{
				Vector coldata=new Vector();
				BufferedReader in=new BufferedReader(new FileReader(inf+".col"));
				while(in.ready())
					coldata.add(in.readLine());
				in.close();
				PrintWriter out=new PrintWriter(new FileWriter(outf+".col"));
				for(int i=0; i<coldata.size(); i++) {
					String[] cur=((String)coldata.get(i)).split("\\s",2);
					int idx=Integer.parseInt(cur[0]);
					if(idx==oldi)
						out.println(newi+" "+cur[1]);
					else
						out.println(cur[0]+" "+cur[1]);
				}
				out.close();
			}
				
		} catch (Exception ex) { ex.printStackTrace(); }
		
	}
}
