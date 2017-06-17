package examples;


import ev3dev.actuators.lego.motors.Motor;
import ev3dev.sensors.ev3.EV3IRSensor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.*;
import lejos.robotics.pathfinding.*;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

import java.util.ArrayList;
import java.util.Iterator;

 
public class Sturing {
    static RobotPilot pilot;
	//static WheeledChassis chassis;
	static Path currentPath;
	static Navigator kapitein;
	static PoseProvider ppv;
	static LineMap currentMap;
	static FourWayGridMesh currentMesh;
	static float gridSpace;
	static float clearance = 1;  //moet nog aangepast worden
	static SearchAlgorithm alg = new AstarSearchAlgorithm();
	static Pose currentPose = new Pose (45f,0f,0f);;
	static Waypoint goal;
	static IRSensor sensor;
	static boolean destinationReached = false;
	static boolean obstacleDetected = false;
	
	
	
	/** 
	 * 
	 * WAT MOET NOG GEIMPLEMENTEERD WORDEN:  
	 * @return
	 */
	
	static ArrayList<float[]> getWaypoints () {  /** geeft Arraylist van [x,y] coordinaten van waypoints van current path**/
		ArrayList<float[]> waypoints = new ArrayList<float[]>();
		for (Waypoint waypoint : currentPath) {
			waypoints.add(new float[]{waypoint.x, waypoint.y});
		}
		/*for (int i = 0; i<waypoints.size(); i++) {
			System.out.println("x"+i+" = "+waypoints.get(i)[0]);
			System.out.println("y"+i+" = "+waypoints.get(i)[1]+"\n");
		}*/
		return waypoints;
	}
	
	static void createPilot () {
		//Wheel leftwheel = WheeledChassis.modelWheel(Motor.B, 54.8).invert(true).offset(76);
		//Wheel rightwheel = WheeledChassis.modelWheel(Motor.C, 54.8).invert(true).offset(-76);
		//chassis = new WheeledChassis(new Wheel[] {leftwheel,  rightwheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new RobotPilot(chassis, Motor.A);
		pilot.setAngularSpeed(100);
		pilot.setLinearSpeed(100);
	}
	
	static void createNavigator () {
		ppv = new OdometryPoseProvider(pilot);
		ppv.setPose(currentPose);
		kapitein = new Navigator(pilot,ppv);
	}
	
	static void updatePose() {
		//moet aangevuld worden met code van imageprocessing
		currentPose = new Pose(/*hier komt code van imageprocessing*/);
		kapitein.getPoseProvider().setPose(currentPose);
	}
	
	static void updateMap(float width, float height, ArrayList<float[][]> contours) {
		Rectangle boundingRect = new Rectangle(0,0,width,height);
		ArrayList<Line> lines = new ArrayList<Line>();
		for (Iterator<float[][]> iterator = ( contours.iterator()); iterator.hasNext();) {  // voor elke contour
			ArrayList<float[]> points = new ArrayList<float[]>();
			float[][] contour = iterator.next();
			for (int i = 0; i < contour.length; i++) {
				points.add(new float[]{contour[i][0], contour[i][1]});
			}	
			for (int i = 0; i < points.size()-1; i++) {
				lines.add(new Line(points.get(i)[0], points.get(i)[0], points.get(i+1)[0], points.get(i+1)[1]));
			}
			lines.add(new Line(points.get(0)[0], points.get(0)[1], points.get(points.size()-1)[0], points.get(points.size()-1)[1]));
		}
		/*for (int i = 0; i < lines.size(); i++) {
			System.out.println(lines.get(i).x1);
			System.out.println(lines.get(i).y1);
			System.out.println(lines.get(i).x2);
			System.out.println(lines.get(i).y2);
			System.out.println("\n"); */
		currentMap = new LineMap(lines.toArray(new Line[lines.size()]), boundingRect);
		gridSpace = (width+height)/20;
		
		updatePose();
	}
	
	static void updateMesh () {
		currentMesh = new FourWayGridMesh(currentMap, gridSpace, clearance);
	}
	
	static void updatePath () throws DestinationUnreachableException {
		NodePathFinder padvinder = new NodePathFinder(alg, currentMesh);
		currentPath = padvinder.findRoute(currentPose, goal);
	}
	
	static void updatePath (float width, float height, ArrayList<float[][]> contouren) throws DestinationUnreachableException {
		updateMap(width, height, contouren);
		updateMesh();
		updatePath();
	}
	
	public static void main(String[] args) 	  {
		 createPilot();
		 System.out.println("Pilot created");
		 pilot.setLinearSpeed(100);
		 pilot.setAngularSpeed(100);
		 //pilot.rotate(30);
		 
		 //float[][] boundingPoints = new float[][]{{11f,5f},{105f,5f},{105f,115f},{11f,115f}};
		 ArrayList<float[][]> contouren = new ArrayList<float[][]>();
		 contouren.add(new float[][]{{175f, 200f},{300f, 225f},{250f, 325f},{100f,225f}});
		 goal = new Waypoint(new lejos.robotics.geometry.Point(400f, 400f));
		 createNavigator();
		 System.out.println("Navigator created");
		 updateMap(575f,550f,contouren);
		 updateMesh();
		 try {
			updatePath();
			System.out.println("Path updated");
		} catch (DestinationUnreachableException e) {
			System.out.println("Destination of robot is unreachable");
		}
		 kapitein.setPath(currentPath);
		 sensor = new IRSensor();
		 Behavior b1 = new DoPath();
		 Behavior b2 = new DetectObstacle();
		 Arbitrator arbitrator = new Arbitrator (new Behavior[] {b1});
		 arbitrator.go();
		 
		 
		 
	  }
}

class RobotPilot extends MovePilot {
	RegulatedMotor sensorMotor;
	RotationListener listener = new RotationListener();
	public RobotPilot(Chassis chassis, RegulatedMotor sensorMotor) {
		super(chassis);
		this.sensorMotor=sensorMotor;
		super.addMoveListener(listener);
	}
	public void rotateSensor(int angle) {
		sensorMotor.rotate(angle);
	}
	
	class RotationListener implements MoveListener {
		private int angle=45;    // sensor zal over 45 graden draaien
		float sign;
		public void moveStarted(Move event, MoveProvider mp) {
			if (event.getMoveType().equals(Move.MoveType.ROTATE)){
				sign = Math.signum(event.getAngleTurned());
				rotateSensor((int)(sign*angle));
			}
			else if (event.getMoveType().equals(Move.MoveType.ARC)){
				sign = Math.signum(event.getArcRadius());
				rotateSensor((int)(sign*angle));}			
			;
		}
		public void moveStopped(Move event, MoveProvider mp) {
			rotateSensor(-(int)(sign*angle));;			
		}
	}		
}

class DoPath implements Behavior {
	boolean _surpressed;
	public void action() {
		while (!Sturing.kapitein.pathCompleted()&&_surpressed) {
			Sturing.kapitein.followPath();
			//Thread.yield();
			Sturing.obstacleDetected=Sturing.sensor.getDistance()<3;   //object op minder dan 3 cm van sensor
		}
		System.out.println("Destination reached");
		Sturing.kapitein.clearPath();
		Sturing.destinationReached=true;
		suppress();
	}
	
	public void suppress() {
		
	}
	
	public boolean takeControl() {
		return !Sturing.destinationReached;
	}

}

class DetectObstacle implements Behavior {
	
	public void action() {
		//take new picture and get new bounding points en contouren
		//make new image with detected object
		//Pilot.updateMap(width, height, contours);
		//...
		
		/*try {
			//Pilot.updatePath();  met code van imageProcessing			
		} catch (DestinationUnreachableException e) {
		
		}*/

		System.out.println("Object Detected");
	}
	
	public void suppress() { //will never be called
	}
	
	public boolean takeControl() {
		return Sturing.obstacleDetected;
	}
	
}

class IRSensor
{
	EV3IRSensor ir;
    SampleProvider sp = ir.getDistanceMode();
    private float distance;

    IRSensor()
    {
    	ir = new EV3IRSensor(SensorPort.S1);

    }
    
    public float getDistance(){
    	float [] sample = new float[sp.sampleSize()];
        sp.fetchSample(sample, 0);
        distance = sample[0];
        return distance;
    }
        
        
    
    
}


