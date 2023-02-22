
public class Rectangle {
	public double lengthX;
	public double lengthY;
	public int ID;
	public int totalInventory; 
	
	public Rectangle(int ID, double x, double y) {
		this.ID = ID;
		lengthX = x;
		lengthY = y;
	}

	public double getLengthX() {
		return lengthX;
	}

	public void setLengthX(double x) {
		lengthX = x;
	}

	public double getLengthY() {
		return lengthY;
	}

	public void setLengthY(double y) {
		lengthY= y;
	}

	public int getID() {
		return ID;
	}

	public void setID(int ID) {
		this.ID = ID;
	}
	
	public int getTotalInventory() {
		return totalInventory;
	}
	
	public void setTotalInventory(int total) { // Total inventory of SKU that belongs to the class
		totalInventory = total;
	}
	
}
