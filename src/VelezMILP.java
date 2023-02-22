import java.util.ArrayList;
import java.util.Arrays;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.UnknownObjectException;

public class VelezMILP {
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
	
	private double [] SKULength;
	private int [] numLevels;
	private int maxNumLevels;
	private int [][] maxCapLevels;
	private double [] fractionCap;
	
	// Variables:
	
	private IloIntVar [][] y;
	private IloIntVar [][][] z;
	private IloIntVar [][][][] x;
	
	public VelezMILP(Instance instancia) {
		FSrunTime = 0;
		FSobjectiveValue = 0;
		FSbestBound = 0;
		FSGAP = 0;
		FSnumTrucksUsed = 0;
		SSobjectiveValue = 0;
		numVariables = 0;
		loadParameters(instancia);
	}
	
	public void solveModel() throws UnknownObjectException, IloException {
		try {
			// Create and solve first-stage MILP model: Maximization problem
			createFirstStageModel();
			modelo.setParam(IloCplex.Param.TimeLimit,300); // Maximum run-time of 5 minutes
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
			modelo.setParam(IloCplex.Param.TimeLimit,300); // Maximum run-time of 5 minutes
			start = modelo.getCplexTime();
			if (modelo.solve()) {
				SSrunTime = modelo.getCplexTime()-start;
				SSobjectiveValue = (int) modelo.getObjValue();
				SSbestBound = modelo.getBestObjValue();
				SSGAP = modelo.getMIPRelativeGap();
				printOptimalSolution();
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
		
		y = new IloIntVar [numTrucks][numRoutes];	
		for(int t=0;t<numTrucks;t++)
			for(int r=0;r<numRoutes;r++) {
				y[t][r] = modelo.intVar(0,1,"W"+t+r);
				numVariables++;
			}
		
		x = new IloIntVar [numSKU][numTrucks][maxNumLevels][numRoutes];
		for(int i=0;i<numSKU;i++)
			for(int t=0;t<numTrucks;t++)
				for(int l=0;l<numLevels[t];l++)
					for(int r=0;r<numRoutes;r++) {
						x[i][t][l][r] = modelo.intVar(0,Integer.MAX_VALUE,"X"+i+t+l+r);
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
		
		for(int r=0;r<numRoutes;r++) // (3) Maximum capacity
			for(int t=0;t<numTrucks;t++) 
				for(int l=0;l<numLevels[t];l++) { 
					for(int i=0;i<numSKU;i++) 
						exp.addTerm(SKULength[i], x[i][t][l][r]);
					
					exp.addTerm(-maxCapLevels[t][l], y[t][r]);
					modelo.addLe(exp,0);
					exp.clear();
				}
		
		for(int r=0;r<numRoutes;r++) // (4) Minimum capacity
			for(int t=0;t<numTrucks;t++) {
				for(int l=0;l<numLevels[t];l++) 
					for(int i=0;i<numSKU;i++) 
						exp.addTerm(SKULength[i], x[i][t][l][r]);
					
				for(int l=0;l<numLevels[t];l++) 
					exp.addTerm(-fractionCap[t]*maxCapLevels[t][l], y[t][r]);
				
				modelo.addGe(exp,0);
				exp.clear();
			}

		
		for(int i=0;i<numSKU;i++) // (5) Demand - redundant 
			for(int r=0;r<numRoutes;r++) {
				for(int t=0;t<numTrucks;t++)
					for(int l=0;l<numLevels[t];l++)
						exp.addTerm(1, x[i][t][l][r]);
				
				int demand = 0;
				for(int k=0;k<numDealers[r];k++)
					demand = demand + d[i][r][k];
				
				modelo.addLe(exp, demand);
				exp.clear();
			}
		
		for(int i=0;i<numSKU;i++) // (6) Demand
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++)
					modelo.addLe(z[i][r][k], d[i][r][k]);
		
		for(int i=0;i<numSKU;i++) // (7) Balance 
			for(int r=0;r<numRoutes;r++) {
				for(int t=0;t<numTrucks;t++)
					for(int l=0;l<numLevels[t];l++)
						exp.addTerm(1, x[i][t][l][r]);
				
				for(int k=0;k<numDealers[r];k++)
					exp.addTerm(-1, z[i][r][k]);
				
				modelo.addEq(exp, 0);
				exp.clear();
			}
		
		for(int i=0;i<numSKU;i++) { // (8) Inventory
			for(int r=0;r<numRoutes;r++)
				for(int t=0;t<numTrucks;t++)
					for(int l=0;l<numLevels[t];l++)
						exp.addTerm(1, x[i][t][l][r]);
			
			modelo.addLe(exp, I[i]);
			exp.clear();
		}
		
		for(int c=0;c<numCustomers;c++) { // (9) Credit
			for(int i=0;i<numSKU;i++)
				for(int r=0;r<numRoutes;r++)
					for(int k=0;k<numDealers[r];k++)
						exp.addTerm(p[i]*a[k][r][c], z[i][r][k]);
			
			modelo.addLe(exp, u[c]);
			exp.clear();
		}
		
		for(int t=0;t<numTrucks;t++) { // (10) Truck assignment
			for(int r=0;r<numRoutes;r++)
				exp.addTerm(1, y[t][r]);
			
			modelo.addLe(exp,1);
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
				
		
		y = new IloIntVar [numTrucks][numRoutes];	
		for(int t=0;t<numTrucks;t++)
			for(int r=0;r<numRoutes;r++) 
				y[t][r] = modelo.intVar(0,1,"W"+t+r);
		
		x = new IloIntVar [numSKU][numTrucks][maxNumLevels][numRoutes];
		for(int i=0;i<numSKU;i++)
			for(int t=0;t<numTrucks;t++)
				for(int l=0;l<numLevels[t];l++)
					for(int r=0;r<numRoutes;r++) 
						x[i][t][l][r] = modelo.intVar(0,Integer.MAX_VALUE,"X"+i+t+l+r);
				
		// Objective Function:
		
		IloLinearNumExpr exp = modelo.linearNumExpr();
		
		for(int r=0;r<numRoutes;r++)
			for(int t=0;t<numTrucks;t++) 
				exp.addTerm(1, y[t][r]);

		modelo.addMinimize(exp);
		exp.clear();
		
		// Constraints:
		
		for(int r=0;r<numRoutes;r++) // (3) Maximum capacity
			for(int t=0;t<numTrucks;t++) 
				for(int l=0;l<numLevels[t];l++) { 
					for(int i=0;i<numSKU;i++) 
						exp.addTerm(SKULength[i], x[i][t][l][r]);
					
					exp.addTerm(-maxCapLevels[t][l], y[t][r]);
					modelo.addLe(exp,0);
					exp.clear();
				}
		
		for(int r=0;r<numRoutes;r++) // (4) Minimum capacity
			for(int t=0;t<numTrucks;t++) {
				for(int l=0;l<numLevels[t];l++) 
					for(int i=0;i<numSKU;i++) 
						exp.addTerm(SKULength[i], x[i][t][l][r]);
					
				for(int l=0;l<numLevels[t];l++) 
					exp.addTerm(-fractionCap[t]*maxCapLevels[t][l], y[t][r]);
				
				modelo.addGe(exp,0);
				exp.clear();
			}

		
		for(int i=0;i<numSKU;i++) // (5) Demand - redundant 
			for(int r=0;r<numRoutes;r++) {
				for(int t=0;t<numTrucks;t++)
					for(int l=0;l<numLevels[t];l++)
						exp.addTerm(1, x[i][t][l][r]);
				
				int demand = 0;
				for(int k=0;k<numDealers[r];k++)
					demand = demand + d[i][r][k];
				
				modelo.addLe(exp, demand);
				exp.clear();
			}
		
		for(int i=0;i<numSKU;i++) // (6) Demand
			for(int r=0;r<numRoutes;r++)
				for(int k=0;k<numDealers[r];k++)
					modelo.addLe(z[i][r][k], d[i][r][k]);
		
		for(int i=0;i<numSKU;i++) // (7) Balance 
			for(int r=0;r<numRoutes;r++) {
				for(int t=0;t<numTrucks;t++)
					for(int l=0;l<numLevels[t];l++)
						exp.addTerm(1, x[i][t][l][r]);
				
				for(int k=0;k<numDealers[r];k++)
					exp.addTerm(-1, z[i][r][k]);
				
				modelo.addEq(exp, 0);
				exp.clear();
			}
		
		for(int i=0;i<numSKU;i++) { // (8) Inventory
			for(int r=0;r<numRoutes;r++)
				for(int t=0;t<numTrucks;t++)
					for(int l=0;l<numLevels[t];l++)
						exp.addTerm(1, x[i][t][l][r]);
			
			modelo.addLe(exp, I[i]);
			exp.clear();
		}
		
		for(int c=0;c<numCustomers;c++) { // (9) Credit
			for(int i=0;i<numSKU;i++)
				for(int r=0;r<numRoutes;r++)
					for(int k=0;k<numDealers[r];k++)
						exp.addTerm(p[i]*a[k][r][c], z[i][r][k]);
			
			modelo.addLe(exp, u[c]);
			exp.clear();
		}
		
		for(int t=0;t<numTrucks;t++) { // (10) Truck assignment
			for(int r=0;r<numRoutes;r++)
				exp.addTerm(1, y[t][r]);
			
			modelo.addLe(exp,1);
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
			for(int r=0;r<numRoutes;r++) {
				FSVariableValue[m] = modelo.getValue(y[t][r]);
				if(FSVariableValue[m] > 0.1)
					FSnumTrucksUsed++;
				m++;
			}
		
		for(int i=0;i<numSKU;i++)
			for(int t=0;t<numTrucks;t++)
				for(int l=0;l<numLevels[t];l++)
					for(int r=0;r<numRoutes;r++) {
						FSVariableValue[m] = modelo.getValue(x[i][t][l][r]);
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
			for(int r=0;r<numRoutes;r++) {
				variable[m] = y[t][r];
				m++;
			}
		
		for(int i=0;i<numSKU;i++)
			for(int t=0;t<numTrucks;t++)
				for(int l=0;l<numLevels[t];l++)
					for(int r=0;r<numRoutes;r++) {
						variable[m] = x[i][t][l][r];
						m++;
					}
		
		modelo.addMIPStart(variable, FSVariableValue);
	}
	
	public void printOptimalSolution() throws UnknownObjectException, IloException {
		System.out.println("Optimal solution: ");

		for(int r=0;r<numRoutes;r++) 
			for(int t=0;t<numTrucks;t++)
				if(modelo.getValue(y[t][r])>0.1)
					System.out.println("Route: " +r+" - Truck: " +t);
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
		SKULength = instancia.getSKUXlength();
		
		numLevels = instancia.getNumLevels();
		maxNumLevels = instancia.getMaxNumLevels();
		maxCapLevels = instancia.getMaxCapLevels();
		fractionCap = instancia.getFractionCap();

	}

}

