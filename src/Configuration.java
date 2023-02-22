import java.util.ArrayList;

public class Configuration {
	private int truck;
	private int ID;
	private ArrayList <RectangleLoaded> rectanglesLoaded;
	private int [] numRectanglesLoaded; 
	
	public Configuration (int t, int numRectangles, int id) {
		truck = t;
		rectanglesLoaded = new ArrayList <RectangleLoaded>();
		numRectanglesLoaded = new int [numRectangles];	
		ID = id;
	}
	
	public void add(RectangleLoaded rectangle) {
		rectanglesLoaded.add(rectangle);
		if(rectangle.getID()>=0) 
			numRectanglesLoaded[rectangle.getID()]++;
	}
	
	public void setSideSpace(int ID, boolean value) {
		rectanglesLoaded.get(ID).setSideSpace(value);
	}
	
	public ArrayList <RectangleLoaded> getRectanglesLoaded(){
		return rectanglesLoaded;
	}
	
	public void setRectanglesLoaded(ArrayList <RectangleLoaded> rectangles) {
		rectanglesLoaded = rectangles;
	}
	
	public int getNumberRectanglesLoaded() {
		return rectanglesLoaded.size();
	}
	
	public RectangleLoaded getRectangleLoaded(int ID){
		return rectanglesLoaded.get(ID);
	}
	
	public int numRectanglesLoaded(int ID) {
		return numRectanglesLoaded[ID];
	}
	
	public int getTruckID() {
		return truck;
	}
	
	public int getConfigurationID() {
		return ID;
	}
}
