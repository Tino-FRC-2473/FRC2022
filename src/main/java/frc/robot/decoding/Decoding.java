package frc.robot.decoding;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.StringTokenizer;

public class Decoding {

    private static String toDecode = ""; //"X\nD12.32\nA45.93\nE\nX\nD11.21\nA48.34\nE\nX\nD11.34\n";
    private ArrayList<Double[]> info;

    public Decoding() {
        info = new ArrayList<>();
    }

    public static void main(String[] args) {
        Decoding dec = new Decoding();
        dec.decode(toDecode);
        for(int i=0; i<dec.info.size(); i++) {
            System.out.println(Arrays.toString(dec.info.get(i)));
        }

    }

    public void decode(String s) {
        double dist = 0;
        double angle = 0;
        int pos = s.indexOf('E');
        while(pos!=-1) {
            String vals = s.substring(0, pos+1);
            StringTokenizer st = new StringTokenizer(vals, "\n");
            while(st.hasMoreTokens()) {
                String tok = st.nextToken();
                if(tok.equals("X") || tok.equals("E")) continue;
                if(tok.charAt(0)=='D') dist = Double.parseDouble(tok.substring(1));
                else if(tok.charAt(0)=='A') angle = Double.parseDouble(tok.substring(1));
            }
            Double[] token = {dist, angle};
            info.add(token);
            s = s.substring(pos+1);
            pos = s.indexOf('E');
        }
    }

    public Double getDist(int index) {
        return info.get(index)[0];
    }

    public Double getAngle(int index) {
        return info.get(index)[1];
    }

    public static void addInfo(String s) {
        toDecode += s;
    }

}

