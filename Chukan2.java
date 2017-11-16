package chukan;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
import lejos.hardware.motor.Motor;
import lejos.robotics.RegulatedMotor;
import java.util.*;

public class Chukan2 {
	
	// �g�p���郂�[�^�[�̒�`
	static RegulatedMotor leftMoter = Motor.C;
	static RegulatedMotor rightMoter = Motor.B;
	
	//�g�p����Z���T�[�̒�`
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
	
	// �J���[�Z���T�[�̑O�̕΍�
	static float preDiff = 0;
	static float diff = 0;
	
	static float DELTA_T = 0.005f;
	
	// ���̂�����
	static float BLACK_VALUE = 0.15f;
	static float WHITE_VALUE = 0.8f;
	
	//static float integral = 0.0f;
	
	// I�̐ϕ��p
	static List<Float> integralList = new ArrayList<Float>();
	
	static float leftMotorVal;
	static float rightMotorVal;
	
	public static int CLOCKWISE = 1;
	public static int COUNTERCLOCKWISE = -1;
	
	public static void main(String[] args) {
		
		// ���[�^�[�p�x������
		motor_init();
		
		//�Z���T�[���[�h�̐ݒ�
		SensorMode color = colorSensor.getMode(1);
		//�Z���T�[�̎擾�l���i�[����z��̗p��
		float valueCol[] = new float[color.sampleSize()];
		
		//�Z���T�[���[�h�̐ݒ�
		SensorMode gyro = gyroSensor.getMode(1);
		//�Z���T�[�̎擾�l���i�[����z��̗p��
		float valueGyro[] = new float[gyro.sampleSize()];
		//�W���C���Z���T�[�̃��Z�b�g
		gyroSensor.reset();
		
		//�摜�̏�����
		LCD.clear();
		
		final int spd = 300;
		
		leftMotorVal = spd;
		rightMotorVal = spd;
		
		// �����m
		while(valueCol[0] == 0.0f || valueCol[0] > BLACK_VALUE){
			motor_set(200, 200);
			
			//�Z���T�[�l��p�ӂ����z��Ɋi�[
			color.fetchSample(valueCol, 0);
			
			//�Z���T�[�lLCD�\������
			for(int k=0; k<color.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueCol[k] + "m", 1, k+1);
			}
			
			Delay.msDelay(10);
			LCD.refresh();
		}
		
		// �����m
		while(valueCol[0] < WHITE_VALUE){
			motor_set(200, 200);
			
			//�Z���T�[�l��p�ӂ����z��Ɋi�[
			color.fetchSample(valueCol, 0);
			
			//�Z���T�[�lLCD�\������
			for(int k=0; k<color.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueCol[k] + "m", 1, k+1);
			}
			
			Delay.msDelay(10);
			LCD.refresh();
		}
		
		Delay.msDelay(500);
		motor_set(0, 0);
		Delay.msDelay(500);
		
		while(!Button.ESCAPE.isDown() && valueGyro[0] > -85){
			motor_set(200, -200);
			//�Z���T�[�̃��[�h��\��
			LCD.drawString(gyro.getName(), 1, 0);
			//�Z���T�[�l��z��Ɋi�[
			gyro.fetchSample(valueGyro, 0);
			//�Z���T�[�lLCD�\������
			for(int k=0; k<gyro.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueGyro[k] + "m", 1, k+1);
			}
			Delay.msDelay(10);
			LCD.refresh();
		}
		
		motor_set(0, 0);
		Delay.msDelay(500);
		
		lineTracePIDInit();
		
		while(!Button.ESCAPE.isDown() && valueGyro[0] > -90 - 360){
			
			//�Z���T�[�l��p�ӂ����z��Ɋi�[
			color.fetchSample(valueCol, 0);
			
			float nowColorVal = valueCol[0];
			
			//�Z���T�[�lLCD�\������
			for(int k=0; k<color.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueCol[k] + "m", 1, k+1);
			}
			
			lineTracePID(spd, nowColorVal, BLACK_VALUE, COUNTERCLOCKWISE);
			
			//�Z���T�[�l��z��Ɋi�[
			gyro.fetchSample(valueGyro, 0);
			
			Delay.msDelay(10);
			LCD.refresh();
		}
		
		motor_set(0, 0);
		Delay.msDelay(500);
		
		while(!Button.ESCAPE.isDown() && valueGyro[0] > -90 - 360 - 180){
			motor_set(200, -200);
			//�Z���T�[�̃��[�h��\��
			LCD.drawString(gyro.getName(), 1, 0);
			//�Z���T�[�l��z��Ɋi�[
			gyro.fetchSample(valueGyro, 0);
			//�Z���T�[�lLCD�\������
			for(int k=0; k<gyro.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueGyro[k] + "m", 1, k+1);
			}
			Delay.msDelay(10);
			LCD.refresh();
		}
		
		motor_set(0, 0);
		Delay.msDelay(500);
		
		lineTracePIDInit();
		
		while(!Button.ESCAPE.isDown() && valueGyro[0] < -90 - 360 - 180 + 365){
			
			//�Z���T�[�l��p�ӂ����z��Ɋi�[
			color.fetchSample(valueCol, 0);
			
			float nowColorVal = valueCol[0];
			
			//�Z���T�[�lLCD�\������
			for(int k=0; k<color.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueCol[k] + "m", 1, k+1);
			}
			
			lineTracePID(spd, nowColorVal, BLACK_VALUE, CLOCKWISE);
			
			//�Z���T�[�l��z��Ɋi�[
			gyro.fetchSample(valueGyro, 0);
			
			Delay.msDelay(10);
			LCD.refresh();
		}
		
		motor_set(0, 0);
		Delay.msDelay(500);
		
		while(!Button.ESCAPE.isDown() && valueGyro[0] < -90 - 360 - 180 + 365 + 70){
			motor_set(-200, 200);
			//�Z���T�[�̃��[�h��\��
			LCD.drawString(gyro.getName(), 1, 0);
			//�Z���T�[�l��z��Ɋi�[
			gyro.fetchSample(valueGyro, 0);
			//�Z���T�[�lLCD�\������
			for(int k=0; k<gyro.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueGyro[k] + "m", 1, k+1);
			}
			Delay.msDelay(10);
			LCD.refresh();
		}
		
		motor_set(0, 0);
		Delay.msDelay(500);
		
		motor_set(200, 200);
		Delay.msDelay(3000);
	}
	
	/**
	 * ���[�^�[������
	 */
	private static void motor_init(){
		leftMoter.resetTachoCount();
		rightMoter.resetTachoCount();
		leftMoter.rotate(0);
		rightMoter.rotate(0);
	}
	
	// PID������
	public static void lineTracePIDInit(){
		// �J���[�Z���T�[�̑O�̕΍�
		preDiff = 0;
		diff = 0;
		// I�̐ϕ��p
		integralList = new ArrayList<Float>();
	}
	
	private static void lineTracePID(float speed, float sensorValue, float targetValue, int ang) {
		
		float nowColorVal = sensorValue;
		
		// �΍�
		preDiff = diff;
		diff = nowColorVal - targetValue;
		
		float tmp = (diff + preDiff) / 2.0f * DELTA_T;
		integralList.add(tmp);
		
		// 10�܂�
		if (integralList.size() >= 5) integralList.remove(0);
		
		float integral = 0.0f;
		
		int size = integralList.size();
		for (int i = 0; i < size; i++) {
			integral+=integralList.get(i) * Math.exp(-(size - i - 1));
		}
		
		float p = 150.0f * diff;
		float i = 120.0f * integral;
		float d = 1.0f * (diff - preDiff) / DELTA_T;
		
		float delta = (p + i + d);
		
		leftMotorVal = speed - ang * delta;
		rightMotorVal = speed + ang * delta;
		
		motor_set((int)(leftMotorVal), (int)(rightMotorVal));
		Delay.msDelay(10);
		LCD.refresh();
	}
	
	private static void motor_set(int l_motor_pow, int r_motor_pow) {
		
		leftMoter.setSpeed(l_motor_pow);
		rightMoter.setSpeed(r_motor_pow);
		
		if (l_motor_pow > 0) {
			leftMoter.forward();
		} else if (l_motor_pow < 0) {
			leftMoter.backward();
		} else {
			leftMoter.stop();
		}
		
		if (r_motor_pow > 0) {
			rightMoter.forward();
		} else if (r_motor_pow < 0) {
			rightMoter.backward();
		} else {
			rightMoter.stop();
		}
	}
}
