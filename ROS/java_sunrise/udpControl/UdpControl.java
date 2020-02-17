package udpControl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
//import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.MatrixBuilder;
import com.kuka.roboticsAPI.geometricModel.math.MatrixRotation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class UdpControl extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	
	private JointTorqueCondition bigJ1;
	private JointTorqueCondition bigJ2;
	private JointTorqueCondition bigJ3;
	private JointTorqueCondition bigJ4;
	private JointTorqueCondition bigJ5;
	private JointTorqueCondition bigJ6;
	private JointTorqueCondition bigJ7;
	
	private DatagramSocket server; 	
	private StateSender sender;
	private Thread output;
		
	private final static double jvel = 0.2;   // relative
	private final static double cvel = 100;   // mm/s   - ?
	private final static int stiffnessZ = 2000;
	private final static int stiffnessX = 2000;
	private final static int stiffnessY = 2000;
	
	private final static String sepPoint = ";";
	private final static String sepNum = " ";
	
	private boolean isContinue;

	public void initialize() {
		// initialize robot
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		
		// update home position
		lbr.setHomePosition(new JointPosition(0,0.175,0,-1.396,0,1.57,0));
		isContinue = true;
		// UDP connection
		try {
			server = new DatagramSocket(30003);
		} catch (SocketException se) {
			getLogger().error("Cannot open UDP socket: " + se);
			isContinue = false;
		}
		sender = new StateSender(lbr);
		if(sender.isConnected()) {
			output = new Thread(sender);
			output.start();
		} else {
			getLogger().error("Cannot establish connection with " + sender.getAddr());
			isContinue = false;
		}	
		// break condition
		bigJ1 = new JointTorqueCondition(JointEnum.J1,-10,10);
		bigJ2 = new JointTorqueCondition(JointEnum.J2,-10,10);
		bigJ3 = new JointTorqueCondition(JointEnum.J3,-10,10);
		bigJ4 = new JointTorqueCondition(JointEnum.J4,-10,10);
		bigJ5 = new JointTorqueCondition(JointEnum.J5,-10,10);
		bigJ6 = new JointTorqueCondition(JointEnum.J6,-10,10);
		bigJ7 = new JointTorqueCondition(JointEnum.J7,-10,10);
	}

	public void run() {
		lbr.move(ptpHome().setJointVelocityRel(jvel));
		
		byte[] receiveData = new byte[2048];         // TODO: enought? 
		
		// adjust complinace 
		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		
		DatagramPacket packet = null;
				
		while(isContinue) {
			if(packet == null) {
				getLogger().info("Waiting for UDP data");
				packet = new DatagramPacket(receiveData, receiveData.length);
			}						
			try{
				server.setSoTimeout(1000);
				server.receive(packet);
			} catch (SocketTimeoutException se) {				
				continue;
			} catch (IOException ex) {
				getLogger().error("UDP receive error: " + ex);
				break;
			} 	
			if(packet.getLength() == receiveData.length) {
				getLogger().warn("Message could be truncated!");
			}
						
			String data = new String(packet.getData()).substring(0,packet.getLength());
			String[] points = data.split(sepPoint);
			for(String p : points) {
				// numbers are separated with whitespace
				getLogger().info(p);
				String[] spos = p.split(sepNum);
				if(spos.length != 7) {
					getLogger().warn("Unexpected message length");
					continue;
				}
				// parse
				double X, Y, Z, Qw, Qx, Qy, Qz; 				
				try {
					X = Double.parseDouble(spos[0]);
					Y = Double.parseDouble(spos[1]);
					Z = Double.parseDouble(spos[2]);
					Qw = Double.parseDouble(spos[3]);
					Qx = Double.parseDouble(spos[4]);
					Qy = Double.parseDouble(spos[5]);
					Qz = Double.parseDouble(spos[6]);
				} catch (NumberFormatException ne) {
					getLogger().error("Wrong number: " + ne);
					continue;
				}	
				
				// make goal frame				
				MatrixRotation rot = quatToMatrix(Qw,Qx,Qy,Qz);
				Vector vec = Vector.of(X,Y,Z);
				Frame nextFrame = new Frame(Transformation.of(vec,rot));
								
				// motion
				LIN line = lin(nextFrame);
				//line.setJointVelocityRel(jvel);
				line.setCartVelocity(cvel);
				//line.setOrientationType(SplineOrientationType.OriJoint);
				line.setMode(impedanceControlMode);
				try {
					lbr.move(line.breakWhen(bigJ1).breakWhen(bigJ2).breakWhen(bigJ3).breakWhen(bigJ4)
							     .breakWhen(bigJ5).breakWhen(bigJ6).breakWhen(bigJ7));
				} catch (CommandInvalidException ce) {
					getLogger().warn("No solution!");
					break;
				}				
			}
			packet = null;
		}	
		server.close();
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		UdpControl app = new UdpControl();
		app.runApplication();
	}
	
	private MatrixRotation quatToMatrix(double w, double x, double y, double z) {
		double sqw = w*w;
		double sqx = x*x;
		double sqy = y*y;
		double sqz = z*z;

		MatrixBuilder mb = new MatrixBuilder();

		// invs (inverse square length) is only required if quaternion is not already normalised
		double invs = 1 / (sqx + sqy + sqz + sqw);
		mb.setElement00(( sqx - sqy - sqz + sqw)*invs) ; // since sqw + sqx + sqy + sqz =1/invs*invs
		mb.setElement11((-sqx + sqy - sqz + sqw)*invs);
		mb.setElement22((-sqx - sqy + sqz + sqw)*invs);

		double tmp1 = x*y;
		double tmp2 = z*w;
		mb.setElement10(2.0 * (tmp1 + tmp2)*invs);
		mb.setElement01(2.0 * (tmp1 - tmp2)*invs);

		tmp1 = x*z;
		tmp2 = y*w;
		mb.setElement20(2.0 * (tmp1 - tmp2)*invs);
		mb.setElement02(2.0 * (tmp1 + tmp2)*invs);

		tmp1 = y*z;
		tmp2 = x*w;
		mb.setElement21(2.0 * (tmp1 + tmp2)*invs);
		mb.setElement12(2.0 * (tmp1 - tmp2)*invs);

		return MatrixRotation.of(mb.toMatrix());
	}

	// Call on exit
	@Override 
	public void dispose () {		
		output.interrupt();
		isContinue = false;
		super.dispose();			
	}	
}
