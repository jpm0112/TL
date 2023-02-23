import java.util.ArrayList;
import java.util.Arrays;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.UnknownObjectException;

public class FullMILP {
	private IloCplex modelo;
	public double FSrunTime, FSobjectiveValue, FSbestBound, FSGAP;
	public int SSobjectiveValue, FSnumTrucksUsed;
	public double SSrunTime, SSbestBound, SSGAP;
	private double [] FSVariableValue;
	public int numVariables;
	
	// Parameters:




	public int numSKU, numRoutes, numCustomers, numTrucks;
	private int [] I, p, u;
	private int [][][] d;
	private double [][][] c; 
	private int [] numDealers;
	private int maxNumDealers;
	private int [][][] a;
	private double [] minCap;
	
	private int [][] b;
	private int [][][] M;
	private int [] Dmax;
	private int [][] DRS;
	
	public ArrayList [] truckConfigurations;
	public ArrayList <Rectangle> rectangles;



	public double [][] config_matrix = new double [1000000][9]; // hay que agrandar el valor para que no se caiga




	private int numRectangles;
	private int [] truckXlength, truckYlength;
	
	private int [] numConfigurations, numLevels;
	private int maxConfigurations, maxConfigurationsPerRoute;
	private double minLength;
	private int [][] maxCapLevels;
	
	// Variables:
	
	private IloIntVar [][][] w;
	private IloIntVar [][][] z;
	private IloIntVar [][][] x;
	
	public FullMILP(Instance instancia, int numConfigurationsPerRoute) {
		FSrunTime = 0;
		FSobjectiveValue = 0;
		FSbestBound = 0;
		FSGAP = 0;
		FSnumTrucksUsed = 0;
		SSobjectiveValue = 0;
		numVariables = 0;
		loadParameters(instancia);
		maxConfigurationsPerRoute = numConfigurationsPerRoute;
		maxConfigurations = maxConfigurationsPerRoute*numRoutes;
		createConfigurations();
	}
	
	public void solveModel() throws UnknownObjectException, IloException {
		try {
			// Create and solve first-stage MILP model: Maximization problem
			createFirstStageModel();
			modelo.setParam(IloCplex.Param.TimeLimit, 3600); // Maximum run-time of 60 minutes
			double start = modelo.getCplexTime();
			if (modelo.solve()) {
				FSrunTime = modelo.getCplexTime()-start;
				FSobjectiveValue = modelo.getObjValue();
				FSbestBound = modelo.getBestObjValue();
				FSGAP = modelo.getMIPRelativeGap();
				saveVariables();
				modelo.end();
			}
			
			// Create and solve second-stage MILP model: Minimization problem
			createSecondStageModel();
			setInitialSolution();
			modelo.setParam(IloCplex.Param.TimeLimit, 3600); // Maximum run-time of 60 minutes
			start = modelo.getCplexTime();
			if (modelo.solve()) {
				SSrunTime = modelo.getCplexTime()-start;
				SSobjectiveValue = (int) modelo.getObjValue();
				SSbestBound = modelo.getBestObjValue();
				SSGAP = modelo.getMIPRelativeGap();
				// printOptimalSolution();
				modelo.end();
			}
			
		}
		catch (IloException e) {System.err.println(e);}
	}

	public void createFirstStageModel() throws IloException {
		modelo = new IloCplex();
		
		// Decision Variables:
		
		z = new IloIntVar [numSKU][numRoutes][maxNumDealers];
		for(int i=0;i<numSKU;i++)
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++) {
					z[i][r][k] = modelo.intVar(0,Integer.MAX_VALUE,"Z"+i+r+k);
					numVariables++;
				}
		
		w = new IloIntVar [numTrucks][maxConfigurations][numRoutes];	
		for(int t=0;t<numTrucks;t++)
			for(int n=0;n<numConfigurations[t];n++)
				for(int r=0;r<numRoutes;r++) {
					w[t][n][r] = modelo.intVar(0,1,"W"+t+n+r);
					numVariables++;
				}
		
		x = new IloIntVar [numSKU][numTrucks][numRoutes];
		for(int i=0;i<numSKU;i++)
			for(int t=0;t<numTrucks;t++)
				for(int r=0;r<numRoutes;r++) {
					x[i][t][r] = modelo.intVar(0,Integer.MAX_VALUE,"X"+i+t+r);
					numVariables++;
				}
		
		// Objective Function:
		
		IloLinearNumExpr exp = modelo.linearNumExpr();
		
		for(int i=0;i<numSKU;i++)
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++)
					exp.addTerm(c[i][r][k], z[i][r][k]);
				
		modelo.addMaximize(exp);
		exp.clear();
		
		// Constraints:
		
		for(int t=0;t<numTrucks;t++) { // (3) Assignment of configurations
			for(int n=0;n<numConfigurations[t];n++)
				for(int r=0;r<numRoutes;r++)
					exp.addTerm(1, w[t][n][r]);
			
			modelo.addLe(exp,1);
			exp.clear();
		}
		
		for(int t=0;t<numTrucks;t++) // (4) Linking W and X variables: Maximum capacity
			for(int s=0;s<numRectangles;s++)
				for(int r=0;r<numRoutes;r++) {
					for(int i=0;i<numSKU;i++) 
						exp.addTerm(x[i][t][r], b[i][s]);
					
					for(int n=0;n<numConfigurations[t];n++)
						exp.addTerm(-M[s][t][n], w[t][n][r]);
					
					modelo.addLe(exp,0);
					exp.clear();
				}
		
		for(int t=0;t<numTrucks;t++)  // (5) Linking W and X variables: Minimum capacity
			for(int r=0;r<numRoutes;r++) { 
				for(int i=0;i<numSKU;i++) 
					exp.addTerm(1, x[i][t][r]);
				
				for(int n=0;n<numConfigurations[t];n++)
					exp.addTerm(-minCap[t], w[t][n][r]);	
				
				modelo.addGe(exp,0);
				exp.clear();
			}
			
		for(int i=0;i<numSKU;i++) // (6) Linking X and Z variables
			for(int r=0;r<numRoutes;r++) {
				for(int t=0;t<numTrucks;t++)
					exp.addTerm(1, x[i][t][r]);
				
				for(int k=0;k<numDealers[r];k++)
					exp.addTerm(-1, z[i][r][k]);
				
				modelo.addEq(exp, 0);
				exp.clear();
			}
		
		for(int i=0;i<numSKU;i++) // (7) Demand
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++)
					modelo.addLe(z[i][r][k], d[i][r][k]);
		
		for(int i=0;i<numSKU;i++) { // (8) Inventory
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++)
					exp.addTerm(1, z[i][r][k]);
			
			modelo.addLe(exp,I[i]);
			exp.clear();
		}
		
		for(int c=0;c<numCustomers;c++) { // (9) Credit
			for(int i=0;i<numSKU;i++)
				for(int r=0;r<numRoutes;r++)
					for(int k=0;k<numDealers[r];k++)
						exp.addTerm(p[i]*a[k][r][c], z[i][r][k]);
			
			modelo.addLe(exp,u[c]);
			exp.clear();
		}
					
	}
	
	public void createSecondStageModel() throws IloException {
		modelo = new IloCplex();
		
		// Decision Variables:
		
		z = new IloIntVar [numSKU][numRoutes][maxNumDealers];
		for(int i=0;i<numSKU;i++)
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++) 
					z[i][r][k] = modelo.intVar(0,Integer.MAX_VALUE,"Z"+i+r+k);
				
		w = new IloIntVar [numTrucks][maxConfigurations][numRoutes];	
		for(int t=0;t<numTrucks;t++)
			for(int n=0;n<numConfigurations[t];n++)
				for(int r=0;r<numRoutes;r++) 
					w[t][n][r] = modelo.intVar(0,1,"W"+t+n+r);
		
		x = new IloIntVar [numSKU][numTrucks][numRoutes];
		for(int i=0;i<numSKU;i++)
			for(int t=0;t<numTrucks;t++)
				for(int r=0;r<numRoutes;r++) 
					x[i][t][r] = modelo.intVar(0,Integer.MAX_VALUE,"X"+i+t+r);
				
		// Objective Function:
		
		IloLinearNumExpr exp = modelo.linearNumExpr();
		
		for(int r=0;r<numRoutes;r++)
			for(int t=0;t<numTrucks;t++) 
				for(int n=0;n<numConfigurations[t];n++)
					exp.addTerm(1, w[t][n][r]);

		modelo.addMinimize(exp);
		exp.clear();
		
		// Constraints:
				
		for(int t=0;t<numTrucks;t++) { // (3) Assignment of configurations
			for(int n=0;n<numConfigurations[t];n++)
				for(int r=0;r<numRoutes;r++)
					exp.addTerm(1, w[t][n][r]);
			
			modelo.addLe(exp,1);
			exp.clear();
		}
		
		for(int t=0;t<numTrucks;t++) // (4) Linking W and X variables: Maximum capacity
			for(int s=0;s<numRectangles;s++)
				for(int r=0;r<numRoutes;r++) {
					for(int i=0;i<numSKU;i++) 
						exp.addTerm(x[i][t][r], b[i][s]);
					
					for(int n=0;n<numConfigurations[t];n++)
						exp.addTerm(-M[s][t][n], w[t][n][r]);
					
					modelo.addLe(exp,0);
					exp.clear();
				}
		
		for(int t=0;t<numTrucks;t++)  // (5) Linking W and X variables: Minimum capacity
			for(int r=0;r<numRoutes;r++) { 
				for(int i=0;i<numSKU;i++) 
					exp.addTerm(1, x[i][t][r]);
				
				for(int n=0;n<numConfigurations[t];n++)
					exp.addTerm(-minCap[t], w[t][n][r]);	
				
				modelo.addGe(exp,0);
				exp.clear();
			}
			
		for(int i=0;i<numSKU;i++) // (6) Linking X and Z variables
			for(int r=0;r<numRoutes;r++) {
				for(int t=0;t<numTrucks;t++)
					exp.addTerm(1, x[i][t][r]);
				
				for(int k=0;k<numDealers[r];k++)
					exp.addTerm(-1, z[i][r][k]);
				
				modelo.addEq(exp, 0);
				exp.clear();
			}
		
		for(int i=0;i<numSKU;i++) // (7) Demand
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++)
					modelo.addLe(z[i][r][k], d[i][r][k]);
		
		for(int i=0;i<numSKU;i++) { // (8) Inventory
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++)
					exp.addTerm(1, z[i][r][k]);
			
			modelo.addLe(exp,I[i]);
			exp.clear();
		}
		
		for(int c=0;c<numCustomers;c++) { // (9) Credit
			for(int i=0;i<numSKU;i++)
				for(int r=0;r<numRoutes;r++)
					for(int k=0;k<numDealers[r];k++)
						exp.addTerm(p[i]*a[k][r][c], z[i][r][k]);
			
			modelo.addLe(exp,u[c]);
			exp.clear();
		}
		
		// Additional constraint:
		
		for(int i=0;i<numSKU;i++)
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++)
					exp.addTerm(c[i][r][k], z[i][r][k]);
				
		modelo.addGe(exp, FSobjectiveValue);
		exp.clear();	
		
	}
	
	public void printOptimalSolution() throws UnknownObjectException, IloException {
		System.out.println("Optimal solution: ");

		for(int r=0;r<numRoutes;r++)
			for(int t=0;t<numTrucks;t++)
				for(int n=0;n<numConfigurations[t];n++)
					if(modelo.getValue(w[t][n][r])>0.1) {
						System.out.println("Route: " +r+" - Truck: " +t);
						
						/*int demandServed = 0;
						for(int i=0;i<numSKU;i++)
							for(int k=0;k<numDealers[r];k++) 
								demandServed = (int) (demandServed + modelo.getValue(z[i][r][k]));
						
						System.out.println("Total demand: " + demandServed);
						
						int SKULoaded = 0;
						for(int i=0;i<numSKU;i++) 
							SKULoaded = (int) (SKULoaded + modelo.getValue(x[i][t][r]));
						
						System.out.println("SKU loaded: "+ SKULoaded);
						System.out.println();*/
					}
	}
	
	public void saveVariables() throws UnknownObjectException, IloException {	
		FSVariableValue = new double [numVariables];
		
		int m = 0;
		for(int i=0;i<numSKU;i++)
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++) {
					FSVariableValue[m] = modelo.getValue(z[i][r][k]);
					m++;
				}
		
		for(int t=0;t<numTrucks;t++)
			for(int n=0;n<numConfigurations[t];n++)
				for(int r=0;r<numRoutes;r++) {
					FSVariableValue[m] = modelo.getValue(w[t][n][r]);
					if(FSVariableValue[m] > 0.1)
						FSnumTrucksUsed++;
					m++;
				}
		
		for(int i=0;i<numSKU;i++)
			for(int t=0;t<numTrucks;t++)
				for(int r=0;r<numRoutes;r++) {
					FSVariableValue[m] = modelo.getValue(x[i][t][r]);
					m++;
				}
	}
	
	public void setInitialSolution() throws IloException {
		IloNumVar [] variable =  new IloNumVar[numVariables];
		
		int m = 0;
		for(int i=0;i<numSKU;i++)
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++) {
					variable[m] = z[i][r][k];
					m++;
				}
		
		for(int t=0;t<numTrucks;t++)
			for(int n=0;n<numConfigurations[t];n++)
				for(int r=0;r<numRoutes;r++) {
					variable[m] = w[t][n][r];
					m++;
				}
		
		for(int i=0;i<numSKU;i++)
			for(int t=0;t<numTrucks;t++)
				for(int r=0;r<numRoutes;r++) {
					variable[m] = x[i][t][r];
					m++;
				}
		
		modelo.addMIPStart(variable, FSVariableValue);
	}
	
	public void createConfigurations() {
		truckConfigurations = new ArrayList[numTrucks];
		M = new int [numRectangles][numTrucks][maxConfigurations];
		numConfigurations = new int [numTrucks];
		Arrays.fill(numConfigurations, maxConfigurations);
		int counter = 0;

		System.out.println(config_matrix);
		for(int truck=0;truck<numTrucks;truck++) {

			int Y = truckYlength[truck];
			ArrayList <Configuration> configurationSet = new ArrayList <Configuration>();
			
			// Create configurations for each route:
			for(int route=0;route<numRoutes;route++){ 
				
				for(int configurationNumber=0;configurationNumber<maxConfigurationsPerRoute;configurationNumber++) {
					Configuration configuration = new Configuration(truck, numRectangles, configurationNumber + route*maxConfigurationsPerRoute);
					
					for(int level=0;level<numLevels[truck];level++) {
						
						// Initialization:
						int numFirstRectangleOnCurrentLevel = configuration.getNumberRectanglesLoaded();
						double cornerX = 0;
						
						if(level>0) 
							for(int l=0;l<level;l++)
								cornerX = cornerX + maxCapLevels[truck][l];

						double cornerY = 0;
						double horizontal = maxCapLevels[truck][level]; 
						double vertical = Y;
						RectangleLoaded base = loadRectangle(true, route, cornerX + horizontal, vertical, cornerX, cornerY, configuration, level);  // Fictitious rectangle				
						configuration.add(base);
					
						// Loading heuristic:
						boolean complete = false;

						while (!complete) {	
							RectangleLoaded rectangle = loadRectangle(false, route, horizontal, vertical, cornerX, cornerY, configuration, level);
							if (rectangle!=null) {
								// Load rectangle:
								config_matrix[counter][0] = truck;
								config_matrix[counter][1] = route;
								config_matrix[counter][2] = configurationNumber;
								config_matrix[counter][3] = level;
								config_matrix[counter][4] = cornerX;
								config_matrix[counter][5] = cornerY;
								config_matrix[counter][6] = rectangle.getLengthX();
								config_matrix[counter][7] = rectangle.getLengthY();
								config_matrix[counter][8] = maxCapLevels[truck][level];
								counter = counter +1;

								configuration.add(rectangle);
								cornerY = cornerY + rectangle.getLengthY();
								horizontal = rectangle.getLengthX(); 
								vertical = Y - cornerY;

							}
							else {
								// Return to the next base rectangle:
								int number = configuration.getNumberRectanglesLoaded() - 1;
								boolean identified = false;
								while(!identified && number > numFirstRectangleOnCurrentLevel) {
									rectangle = configuration.getRectangleLoaded(number);
									if (rectangle.hasSideSpace()) {
										identified = true;
										configuration.setSideSpace(number, false);
										cornerX = rectangle.getPosX() + rectangle.getLengthX();
										cornerY = rectangle.getPosY();
										
										int baseNumber = number - 1;
										while(baseNumber > numFirstRectangleOnCurrentLevel && !configuration.getRectangleLoaded(baseNumber).hasSideSpace())
											baseNumber--;
										
										base = configuration.getRectangleLoaded(baseNumber);
										horizontal = base.getLengthX() - cornerX;
										vertical = Y - cornerY;
									}
									else
										number--;
								}
								if (number == numFirstRectangleOnCurrentLevel)	
									complete = true;		
							}
						}
					}

					// Add the new configuration created to the set of configurations:
					configurationSet.add(configuration);
				}
			}
			truckConfigurations[truck] = configurationSet;


		}


		System.out.println("configuraciones guardadas");
		System.out.print("\n");
		System.out.println(config_matrix[0][7]);
		System.out.print("\n");
	}
	
	public RectangleLoaded loadRectangle(boolean isFictitious, int route, double horizontal, double vertical, double cornerX, double cornerY, Configuration configuration, int level) {	
				
		if (isFictitious) 
			return new RectangleLoaded(new Rectangle(-1, horizontal, 0), cornerX, cornerY, 0, false);
		
		ArrayList <Integer> nonDiscardedRectangles = new ArrayList <Integer>();
		for(int i=0;i<numRectangles;i++)
			nonDiscardedRectangles.add(i);

		boolean loaded = false;
		while (!loaded && nonDiscardedRectangles.size()>0 && vertical >= minLength) { 
			
			// Select a non-discarded rectangle:
			int ID = getRandomRectangle(route, nonDiscardedRectangles, configuration);
			
			boolean belongToNonDiscarded = false;
			for (int s=0;s<nonDiscardedRectangles.size() && !belongToNonDiscarded;s++) 
				if(ID==nonDiscardedRectangles.get(s))
					belongToNonDiscarded = true;
			
			if (!belongToNonDiscarded)
				return null;
			
			Rectangle rectangleToLoad = rectangles.get(ID);
			double Sx = rectangleToLoad.getLengthX();
			double Sy = rectangleToLoad.getLengthY();
			
			// Check if the selected rectangle (both horizontally and vertically) fits:
			boolean horizontalOrientation = false;
			if(Sx <= horizontal && Sy <= vertical)
				horizontalOrientation = true;
			
			boolean verticalOrientation = false;
			if(Sy <= horizontal && Sx <= vertical)
				verticalOrientation = true;
			
			// Load the selected rectangle if possible:
			if((!horizontalOrientation && !verticalOrientation) || configuration.numRectanglesLoaded(ID) >= rectangleToLoad.getTotalInventory() || configuration.numRectanglesLoaded(ID) >= DRS[route][ID]) {
				nonDiscardedRectangles.remove(new Integer(ID));
			}
			else {
				loaded = true;	
				M[ID][configuration.getTruckID()][configuration.getConfigurationID()]++;
				if ((!horizontalOrientation && verticalOrientation) || (horizontalOrientation && verticalOrientation && Math.random() > 1.0)) { // 1.0 means fixed orientation
					// Load the rectangle in vertical orientation: 
					if (Sy + minLength <= horizontal)
						return new RectangleLoaded(rectangleToLoad, cornerX, cornerY, 1, true); 
					else
						return new RectangleLoaded(rectangleToLoad, cornerX, cornerY, 1, false); 
				}
				// Load the rectangle in horizontal orientation: 
				if (Sx + minLength <= horizontal)
					return new RectangleLoaded(rectangleToLoad, cornerX, cornerY, 0, true); 
				else
					return new RectangleLoaded(rectangleToLoad, cornerX, cornerY, 0, false);
			}
		} 
		
		return null;
	}
	
	public int getRandomRectangle(int route, ArrayList <Integer> nonDiscardedRectangles, Configuration configuration) {
		// Compute the total demand of the non-discarded rectangles:
		int totalDemand = 0;
		for (int s=0;s<nonDiscardedRectangles.size();s++) 
			totalDemand = totalDemand + DRS[route][nonDiscardedRectangles.get(s)];
		
		// Create probability distribution:
		DistributedRandomNumberGenerator distribution = new DistributedRandomNumberGenerator();
		for (int s=0;s<nonDiscardedRectangles.size();s++) 
			 distribution.addNumber(nonDiscardedRectangles.get(s), (double) DRS[route][nonDiscardedRectangles.get(s)]/totalDemand);
	
		return distribution.getRandomNumber();
	}
	
	public void loadParameters(Instance instancia) {	
		numSKU = instancia.getNumSKU(); 
		numRoutes = instancia.getNumRoutes(); 
		numCustomers = instancia.getNumCustomers();
		numTrucks = instancia.getNumTrucks();		
		I = instancia.getI();
		p = instancia.getP();
		u = instancia.getU();
		d = instancia.getD();
		numDealers = instancia.getNumDealers();
		maxNumDealers = instancia.getMaxNumDealers();
		a = instancia.getA();
		c = instancia.getC();
		b = instancia.getB();
		minLength = instancia.getMinLength();
		truckXlength = instancia.getTruckXlength();
		truckYlength = instancia.getTruckYlength();
		rectangles = instancia.getRectangles();
		numRectangles = rectangles.size();
		Dmax = instancia.getDMax();
		minCap = instancia.getMinCap();
		DRS = instancia.getDemandPerRoute();
		maxCapLevels = instancia.getMaxCapLevels();
		numLevels = instancia.getNumLevels();
	}

}

