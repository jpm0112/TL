
public class RectangleLoaded extends Rectangle{
	private double posX;
	private double posY;
	private int orientation;
	private boolean sideSpace;
	
	public RectangleLoaded(Rectangle classToLoad, double x, double y, int o, boolean space) {
		super(classToLoad.getID(), classToLoad.getLengthX(), classToLoad.getLengthY());
		posX = x;
		posY = y;
		orientation = o;
		if (orientation==1) {
			double Sx = classToLoad.getLengthY();
			double Sy = classToLoad.getLengthX();
			setLengthX(Sx);
			setLengthY(Sy);
		}
		sideSpace = space;
	}
	
	public void setLengthX(double x) {
		lengthX = x;
	}

	public void setLengthY(double y) {
		lengthY= y;
	}

	public double getPosX() {
		return posX;
	}

	public void setPosX(double x) {
		posX = x;
	}

	public double getPosY() {
		return posY;
	}

	public void setPosY(double y) {
		posY = y;
	}

	public int getOrientation() {
		return orientation;
	}

	public void setOrientation(int o) {
		orientation = o;
	}
	
	public boolean hasSideSpace() { // True if the rectangle has another rectangle on the right
		return sideSpace;
	}

	public void setSideSpace(boolean sideSpace) {
		this.sideSpace = sideSpace;
	}
}
