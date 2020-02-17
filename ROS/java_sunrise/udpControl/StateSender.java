package udpControl;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;

public class StateSender implements Runnable {
	private LBR lbr; 
	private DatagramSocket client;
	private InetAddress ipAddr;	
	
	private final static String addr = "172.31.1.2";
	private final static int port = 10000;
	private final static int waitTime = 10; // ms
	
	private boolean connected;
	
	public StateSender (LBR robot) {
		// connection	
		connected = true;
		try {
			client = new DatagramSocket();
			ipAddr = InetAddress.getByName(addr);
			Thread.sleep(500);
		} catch (Exception e) {
			connected = false;
		}
	    lbr = robot;
	}

	@Override
	public void run() {
		while(true) {
			// check exit
			if(Thread.currentThread().isInterrupted()) {
				client.close();
				break;
			}
			// read and convert state
			JointPosition jp = lbr.getCurrentJointPosition();
			TorqueSensorData tau = lbr.getExternalTorque();
			double[] v = jp.get();
			double[] t = tau.getTorqueValues();
			String pos = String.format("%.4f %.4f %.4f %.4f %.4f %.4f %.4f", 
					v[0],v[1],v[2],v[3],v[4],v[5],v[6]);
			pos += String.format(" %.4f %.4f %.4f %.4f %.4f %.4f %.4f", 
					t[0],t[1],t[2],t[3],t[4],t[5],t[6]);
			byte[] data = pos.getBytes();
			// send
			DatagramPacket packet = new DatagramPacket(data,data.length,ipAddr,port);
			try {
				client.send(packet);
				Thread.sleep(waitTime);
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	public boolean isConnected() { return connected; }
	
	public String getAddr() { return addr + ":" + Integer.toString(port); }

}
