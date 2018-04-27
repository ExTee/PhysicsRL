package comp559.a3;

import java.io.File;

import org.encog.Encog;
import org.encog.engine.network.activation.ActivationSigmoid;
import org.encog.engine.network.activation.ActivationReLU;
import org.encog.engine.network.activation.ActivationLinear;
import org.encog.ml.data.MLData;
import org.encog.ml.data.MLDataPair;
import org.encog.ml.data.MLDataSet;
import org.encog.ml.data.basic.BasicMLData;
import org.encog.ml.data.basic.BasicMLDataPair;
import org.encog.ml.data.basic.BasicMLDataSet;
import org.encog.neural.networks.BasicNetwork;
import org.encog.neural.networks.layers.BasicLayer;
import org.encog.neural.networks.training.propagation.resilient.ResilientPropagation;
import org.encog.neural.networks.training.propagation.sgd.StochasticGradientDescent;
import org.encog.neural.networks.training.propagation.sgd.update.MomentumUpdate;
import org.encog.persist.EncogDirectoryPersistence;
import org.encog.util.Format;
import org.encog.neural.networks.training.propagation.back.Backpropagation;
 

public class NeuralNetwork_HeadOnly {
	
	public BasicNetwork network = new BasicNetwork();

	public MLDataSet trainingSet = new BasicMLDataSet();
	StochasticGradientDescent sgd;
	
	
	public NeuralNetwork_HeadOnly(){		
		network.addLayer(new BasicLayer(null,true, 8));
		network.addLayer(new BasicLayer(new ActivationReLU(), true, 48));
		network.addLayer(new BasicLayer(new ActivationReLU(), true, 24));
		network.addLayer(new BasicLayer(new ActivationReLU(), true, 24));
		network.addLayer(new BasicLayer(new ActivationLinear(), false, 1));
		network.getStructure().finalizeStructure();
		network.reset();
		

		
// create a neural network (good for quadratic) ,

//				network.addLayer(new BasicLayer(null,true,2));
//				network.addLayer(new BasicLayer(new ActivationReLU(),true,16));
//				network.addLayer(new BasicLayer(new ActivationReLU(),true,32));
//				network.addLayer(new BasicLayer(new ActivationLinear(),false,1));
//				network.getStructure().finalizeStructure();
//				network.reset();
	}
	
	/**
	 * Adds a data entry to the data set
	 * @param x double[] length 24
	 * @param y double[] length 1
	 */
	public void addData(double[] x, double[] y){
		BasicMLData x_train = new BasicMLData(x);
		BasicMLData y_train = new BasicMLData(y);
		
		trainingSet.add(x_train, y_train);
	}
	public void save(){
		EncogDirectoryPersistence.saveObject(new File("C:/Users/XT/Documents/McGill/COMP559/Assignment3/src/comp559/a3/net/networkData_Head.eg"), network);
		System.out.println("Network Saved");
	}
	
	public void load(){
		network = (BasicNetwork) EncogDirectoryPersistence.loadObject(new File("C:/Users/XT/Documents/McGill/COMP559/Assignment3/src/comp559/a3/net/networkData_Head.eg"));
		System.out.println("Network Loaded");
	}
	
	public void train(int NUM_EPOCHS){
		
		ResilientPropagation train = new ResilientPropagation(network, trainingSet);
		int epoch = 1;
		 
		do {
			train.iteration();
			System.out.println("Epoch #" + epoch + " Error:" + train.getError());
			epoch++;
		} while((train.getError() > 0.01) && (epoch < NUM_EPOCHS));
		
		train.finishTraining();
		trainingSet = new BasicMLDataSet();
		
	}
	public void trainSGD(int NUM_EPOCHS)
	{


		sgd = new StochasticGradientDescent(network, trainingSet);
		sgd.setLearningRate(0.1);
		sgd.setMomentum(0.9);
		sgd.setUpdateRule(new MomentumUpdate());
	}
	
	
	public void gradientStep(double[] x, double[] y)
	{
		double error = Double.POSITIVE_INFINITY;
		
		MLDataPair pair = new BasicMLDataPair(new BasicMLData(x),new BasicMLData(y));
		
		while (error > 0.01){
			
			
			sgd.process(pair);
			sgd.update();
			
			error = network.calculateError(trainingSet);
			System.out.println("Step #" + sgd.getIteration() + ", Step Error: "
	                + Format.formatDouble(sgd.getError(),2) + ", Global Error: "
	                    + Format.formatDouble(error,2));
		}
	}
	
	public double predict(double[] test){
		BasicMLData x_test = new BasicMLData(test);
		
		MLData output = network.compute(x_test);
		return output.getData(0);
	}
	
	public void close(){
		Encog.getInstance().shutdown();
	}
	
	public static void main(final String args[]) {
		//Test by passing in a quadratic
		
		double[][] QUAD_X= { 	{ -2.0,1}, 
									{ -4.0,1},
									{ 1.0,1}, 
									{ 3.0,1},
									{ 0.5,1},
									{5.0,1},
									{ 6,1}, 
									{ 7,1},
									{ 8,1},
									{9,1}
								};
		
		double[][] QUAD_Y= { 	{ 4.0 }, 
								{ 16.0 },
								{ 1.0 }, 
								{ 9.0 },
								{ 0.25},
								{25.0},
								{ 36}, 
								{ 49},
								{ 64},
								{81}
			};		
		
		
		NeuralNetwork_HeadOnly myNet = new NeuralNetwork_HeadOnly();
		for (int i = 0; i < QUAD_X.length; i++){
			myNet.addData(QUAD_X[i], QUAD_Y[i]);
		}
		
		myNet.train(2000);
		
		double out = myNet.predict(new double[]{5.9,1});
		System.out.println(out);
		
		
		myNet.save();
		
		
		NeuralNetwork_HeadOnly newNet = new NeuralNetwork_HeadOnly();
		newNet.load();
		double out2 = newNet.predict(new double[]{7.7,1});
		System.out.println(out2);
		
	}
	
}
