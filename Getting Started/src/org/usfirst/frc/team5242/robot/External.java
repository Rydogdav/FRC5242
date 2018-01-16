package org.usfirst.frc.team5242.robot;

public class External {
	
	public class Auto {
		int i = 0;
		String[][][] sequences = new String[5][15][100];
		public void initSequences ( ) {
			//Conceptually only
			addNewParam("1");
		}
		public void addNewParam (String param) {
			isNumeric(param);
			sequences[0][0][i] = "param"; i++; 
		}
		public boolean isNumeric(String str)  
		{  
		  try  
		  {  
		    double d = Double.parseDouble(str);  
		  }  
		  catch(NumberFormatException nfe)  
		  {  
		    return str;  
		  }  
		  return Integer.parseInt(str);//Doesn't work. Fix!
		}
	}
	
	public class Tele {
		
	}
}
