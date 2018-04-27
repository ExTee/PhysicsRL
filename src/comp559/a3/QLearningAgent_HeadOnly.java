package comp559.a3;
import java.util.Random;
import java.util.Arrays;
import java.util.Collections;


/**
 * Q learning agent for Three part snake
 * @author XT
 *
 */
public class QLearningAgent_HeadOnly {
	
	private int MAX_NUM_STEPS = 1000;
	private double GAMMA = 0.9;
	public int counter = 0;
	

	//public RigidBodySystem sys;
	public Controller c;
	public NeuralNetwork_HeadOnly net;
	
	private double[] previous_SA;
	private double previous_Q;

	
	double magnitude = 2000;
	
	Random r = new Random();
	
	protected double EPSILON = 0.1;
	
	public QLearningAgent_HeadOnly(Controller c)
	{
		//this.sys = sys;
		this.c = c;
		this.net = new NeuralNetwork_HeadOnly();
		
		//Uncomment below to use a pre-trained network. May need further training
		//this.net.load();
	}
	
	public double[] EpsilonGreedy(){
		
		double[] actionTaken = new double[4];
		
		if (Math.random() < this.EPSILON){
			int a = r.nextInt(3 - 0 + 1) + 0;
			
			
			//System.out.println(a);
			switch (a) {
				case 0: c.applyForceUp(magnitude);
						actionTaken = encodeAction(0);
						break;
				case 1: c.applyForceDown(magnitude);
						actionTaken = encodeAction(1);
						break;
				case 2: c.applyForceLeft(magnitude);
						actionTaken = encodeAction(2);
						break;
				case 3: c.applyForceRight(magnitude);
						actionTaken = encodeAction(3);
						break;			
			}
			System.out.println(a);
			
		}
		else{
			//Choose greedy action
			//System.out.println("Chose greedy");
			
			//Obtain projected Q values
			double[] Q_values = queryNet();
			System.out.print(Arrays.toString(Q_values));
			
			int argmax_action = 0;
			double max = Q_values[0];
			boolean equal = false;
			for(int i = 1 ; i < Q_values.length; i++)
			{
				if(Q_values[i] > max)
				{
					max = Q_values[i];
					argmax_action = i;
				}
				else if (Q_values[i] == max){
					equal = true;
				}
			}
			
			if(!equal)
			{
				switch (argmax_action) {
				case 0: c.applyForceUp(magnitude);
						actionTaken = encodeAction(0);
						break;
				case 1: c.applyForceDown(magnitude);
						actionTaken = encodeAction(1);
						break;
				case 2: c.applyForceLeft(magnitude);
						actionTaken = encodeAction(2);
						break;
				case 3: c.applyForceRight(magnitude);
						actionTaken = encodeAction(3);
						break;	
				}
				System.out.println(argmax_action);
			}
			else
			{
				int a = r.nextInt(3 - 0 + 1) + 0;

				System.out.println(a);
				switch (a) {
					case 0: c.applyForceUp(magnitude);
							actionTaken = encodeAction(0);
							break;
					case 1: c.applyForceDown(magnitude);
							actionTaken = encodeAction(1);
							break;
					case 2: c.applyForceLeft(magnitude);
							actionTaken = encodeAction(2);
							break;
					case 3: c.applyForceRight(magnitude);
							actionTaken = encodeAction(3);
							break;	
				}		
			}
			
			
			
		}
	
		return actionTaken;
		
	}
	
	public void move()
	{
		//Get the State Action Pair. We need to store this information to update our network once we get the reward.
		double[] Action = EpsilonGreedy();
		double[] State = {c.body.x.x, c.body.x.y, c.body.v.x, c.body.v.y};
		double[] SA = combine(State, Action);
		previous_SA = SA;
	}
	
	public void update()
	{
		//Get reward
		double reward = c.getHorizontalReward();
		
		double[] tmp = queryNet();
		double V_S_prime = min(tmp);
		double Target = reward + this.GAMMA * V_S_prime;
		
		this.net.addData(previous_SA, new double[] {Target});
		//this.net.gradientStep(previous_SA, new double[] {Target});
		//System.out.println("Data Added");
		this.net.train(200);
		this.net.save();
		//this.counter ++;
		
//		if(this.counter == 5){
//			this.net.train(20);
//			this.net.save();
//			this.counter = 0;
//		}
			
	}
	

	
	/**
	 * Queries the neural network to get value for each action
	 * @return
	 */
	public double[] queryNet(){
		//Current State of the head.
		double[] currentState = {c.body.x.x, c.body.x.y, c.body.v.x, c.body.v.y};
		
		//Values of actions
		double[] Q_values = new double[4];
		
		//Get all possible actions
		for (int action = 0; action < 4; action ++)
		{
			double[] cur_action = encodeAction(action);
			
			//Our input to the neural network
			double[] nn_input = combine(currentState, cur_action);
			System.out.println(Arrays.toString(nn_input));
			
			Q_values[action] = net.predict(nn_input);
		}
		
		return Q_values;
	}
	
	/**
	 * One-Hot encodes the action
	 * @param action
	 * @return one-hot encoded array
	 */
	public double[] encodeAction(int action){
		double[] tmp = {0,0,0,0};
		tmp[action] = 1;
		return tmp;
	}
	
	public double min(double[] array) {
        double min = array[0];
        for (int i = 1; i < array.length; i++) {
            if (array[i] < min) {
                min = array[i];
            }
        }
        return min;
    }
	
	/** Helper to combine two arrays */
    public static double[] combine(double[] a, double[] b){
        int length = a.length + b.length;
        double[] result = new double[length];
        System.arraycopy(a, 0, result, 0, a.length);
        System.arraycopy(b, 0, result, a.length, b.length);
        return result;
    }


	
	
	
}
