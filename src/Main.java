


import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import ilog.concert.*;
import java.io.PrintWriter;
import java.util.Arrays;

public class Main {
	public static void main(String [] arg) throws IOException, IloException {
		PrintWriter writer = new PrintWriter("FullMILP_OneDimension.txt", "UTF-8");
		PrintWriter configuration_writer = new PrintWriter("Configuraciones.txt", "UTF-8");

		for(int instance=1;instance<=1;instance++) {
			for(int configurations=2;configurations<=2;configurations=configurations+2) {
				for(int replication=1;replication<=3;replication++) {
					Instance instancia = new Instance(instance);
					FullMILP MILP = new FullMILP(instancia, configurations);
					MILP.solveModel();
					writer.printf("%d	%d	%d	%d	%d	%d	%d	%.3f	%.3f	%.3f	%d	%.3f	%d	%.3f", instance, configurations, replication, MILP.numSKU, MILP.numRoutes, MILP.numCustomers, MILP.numTrucks, MILP.FSrunTime, MILP.FSobjectiveValue, MILP.FSGAP, MILP.FSnumTrucksUsed, MILP.SSrunTime, MILP.SSobjectiveValue, MILP.SSGAP);
					writer.println();

					configuration_writer.printf("%d  %d",configurations,replication);

					File csvfile = new File("config_matrix.csv");
					FileWriter fileWriter = new FileWriter(csvfile);

					for (double[] data : MILP.config_matrix) {
						StringBuilder line = new StringBuilder();
						for (int i = 0; i < data.length; i++) {
							line.append(data[i]);
							if (i != data.length - 1) {
								line.append(',');
							}
						}
						line.append("\n");
						fileWriter.write(line.toString());
					}
					fileWriter.close();

				}
			}
		}
		writer.close();
		configuration_writer.close();
		
		/*PrintWriter writer = new PrintWriter("VelezMILP_191018.txt", "UTF-8");
		Instance instancia = new Instance();
		VelezMILP modelo = new VelezMILP(instancia);
		modelo.solveModel();
		writer.printf("%d	%d	%d	%d	%.3f	%.3f	%.3f	%d	%.3f	%d	%.3f", modelo.numSKU, modelo.numRoutes, modelo.numCustomers, modelo.numTrucks, modelo.FSrunTime, modelo.FSobjectiveValue, modelo.FSGAP, modelo.FSnumTrucksUsed, modelo.SSrunTime, modelo.SSobjectiveValue, modelo.SSGAP);
		writer.println();	
		writer.close();*/
		
	}
}
