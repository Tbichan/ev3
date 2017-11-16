package finalkadai;

import java.util.ArrayList;
import java.util.List;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import lejos.hardware.motor.Motor;

public class FinalKdai {
	
	// �g�p���郂�[�^�[�̒�`
	static RegulatedMotor leftMoter = Motor.C;
	static RegulatedMotor rightMoter = Motor.B;
	static RegulatedMotor middleMoter = Motor.D;

	//�g�p����Z���T�[�̒�`
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
	static EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
	
	//�Z���T�[���[�h�̐ݒ�
	static SensorMode gyro;
	static SensorMode color;
	static float valueCol[];
	
	static float[] valueGyro;
	
	static SensorMode sonic;
	static float[] valueSonic;
	
	// �J���[�Z���T�[�̑O�̕΍�
	static float preDiff = 0;
	static float diff = 0;
	
	static float DELTA_T = 0.005f;
	
	// ���̂�����
	static float BLACK_VALUE = 0.15f;
	static float WHITE_VALUE = 0.8f;
	static float BLUE_VALUE = 0.05f;
	
	// I�̐ϕ��p
	static List<Float> integralList = new ArrayList<Float>();
	
	// ��Q����臒l
	public static final float THRESHOLD = 0.06f;
	
	public static int CLOCKWISE = 1;
	public static int COUNTERCLOCKWISE = -1;
	
	public static void main(String[] args) {
		
		//�Z���T�[���[�h�̐ݒ�
		color = colorSensor.getMode(1);
		//�Z���T�[�̎擾�l���i�[����z��̗p��
		valueCol = new float[color.sampleSize()];
		
		//�Z���T�[���[�h�̐ݒ�
		sonic = sonicSensor.getMode(0);
		//�Z���T�[�̎擾�l���i�[����z��̗p��
		valueSonic = new float[sonic.sampleSize()];
		
		//�Z���T�[���[�h�̐ݒ�
		gyro = gyroSensor.getMode(1);
		//�Z���T�[�̎擾�l���i�[����z��̗p��
		valueGyro = new float[gyro.sampleSize()];
		//�W���C���Z���T�[�̃��Z�b�g
		gyroSensor.reset();
		
		//�摜�̏�����
		LCD.clear();
		
		// ���[�^�[�p�x������
		motor_init();
		
		int pow = 100;
		int powHigh = 150;
		
		// �O�i�APID
		forwardPID(pow/2, 300, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// ���[�^�[������
		motor_init();
		
		// �O�i
		forward(powHigh+4, powHigh, 360);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		//  �u���b�N��������܂őO�i
		float deg = forwardFindBlockPID(pow, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// �u���b�N������
		catchBlock();
		
		// ��i
		forward(-pow/2, deg + 30);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// ��]
		angle(pow/2, 90, CLOCKWISE);
		
		// �O�i
		forward(powHigh, powHigh+4, 290);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// �O�i�APID
		forwardPID(pow * 2, 1470, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// �O�i
		forward(powHigh, 475);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// ��]�A�덷����H
		angle(pow/2, 78, COUNTERCLOCKWISE);
		
		// �O�i
		forward(powHigh, 100);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// �O�i�APID
		forwardPID(pow/2, 320, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// �͂Ȃ�
		releaseBlock();
		
		// ��i
		forward(-pow/2, 200);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		angle(pow/2, 170, COUNTERCLOCKWISE);
		
		forward(powHigh, 300);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		//  �u���b�N��������܂őO�i
		deg = forwardFindBlockPID(pow/2, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
			
		// �u���b�N������
		catchBlock();
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// ��i�@
		forward(-pow+10, -pow, deg + 200);
		//forward(-pow, deg + 200);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		angle(pow/2, 172, CLOCKWISE);
		
		// �O�i�APID
		forwardPID(pow/3, 320, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// �͂Ȃ�
		releaseBlock();
		
		motor_set(0, 0);
		Delay.msDelay(2000);
		
		// �O�i
		forward(pow, 100);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// ��]
		angle(pow/2, 170, COUNTERCLOCKWISE);
		
		//  �u���b�N��������܂őO�i
		deg = forwardFindBlockPID(pow/2, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// �u���b�N������
		catchBlock();
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// ��]
		angle(pow/2, 170, CLOCKWISE);
		
		// �͂Ȃ�
		releaseBlock();
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// �O�i
		forward(-pow, 100);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
	}
	
	/**
	 * ���[�^�[������
	 */
	private static void motor_init(){
		leftMoter.resetTachoCount();
		rightMoter.resetTachoCount();
		middleMoter.resetTachoCount();
		leftMoter.rotate(0);
		rightMoter.rotate(0);
		middleMoter.rotate(0);
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
	
	// deg�x�S�g
	private static void forward(int motor_pow, float deg) {
		
		// ���[�^�[�p�x������
		motor_init();
		
		// �W���C���Z���T�[�̃��Z�b�g
		gyroSensor.reset();
		
		motor_set(motor_pow, motor_pow);
		
		int val = 3;
		//if (motor_pow < 0) val *= -1;
		
		while((Math.abs(rightMoter.getTachoCount())+Math.abs(leftMoter.getTachoCount()))*0.5f < deg){
			
			// �Z���T�[�l��z��Ɋi�[
			gyro.fetchSample(valueGyro, 0);
			if (valueGyro[0] > 0) motor_set(motor_pow + val, motor_pow - val);
			else motor_set(motor_pow - val, motor_pow + val);
			Delay.msDelay(1);
		}
	}
	
	// deg�x�S�g
	private static void forward(int l_motor_pow, int r_motor_pow, float deg) {
		
		// ���[�^�[�p�x������
		motor_init();
		
		// �W���C���Z���T�[�̃��Z�b�g
		gyroSensor.reset();
		
		motor_set(l_motor_pow, r_motor_pow);
		
		int val = 3;
		//if (motor_pow < 0) val *= -1;
		
		while((Math.abs(rightMoter.getTachoCount())+Math.abs(leftMoter.getTachoCount()))*0.5f < deg){
			
			// �Z���T�[�l��z��Ɋi�[
			gyro.fetchSample(valueGyro, 0);
			if (valueGyro[0] > 0) motor_set(l_motor_pow + val, r_motor_pow - val);
			else motor_set(l_motor_pow - val, r_motor_pow + val);
			Delay.msDelay(1);
		}
	}
	
	// deg�x�S�g
	private static void forwardPID(int motor_pow, float deg, int angle) {
		
		// ���[�^�[�p�x������
		motor_init();
		
		//�W���C���Z���T�[�̃��Z�b�g
		gyroSensor.reset();
		
		motor_set(motor_pow, motor_pow);
		
		lineTracePIDInit();
		
		//�Z���T�[�l��p�ӂ����z��Ɋi�[
		color.fetchSample(valueCol, 0);
		
		while((Math.abs(rightMoter.getTachoCount())+Math.abs(leftMoter.getTachoCount()))*0.5f < deg){
			
			//�Z���T�[�l��p�ӂ����z��Ɋi�[
			color.fetchSample(valueCol, 0);
			
			float nowColorVal = valueCol[0];
			/*
			//�Z���T�[�lLCD�\������
			for(int k=0; k<color.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueCol[k] + "m", 1, k+1);
			}*/
			
			lineTracePID(motor_pow, nowColorVal, BLACK_VALUE, angle);
		}
	}
	
	// �u���b�N�����������ǂ���
	private static boolean isFindBlock() {
		
		//�Z���T�[�l��z��Ɋi�[
		sonic.fetchSample(valueSonic, 0);
	    if (valueSonic[0] <= THRESHOLD) return true;
		
		return false;
	}
	
	// ����
	private static void catchBlock() {
		
		// ���[�^�[�p�x������
		motor_init();
		motor_set(0, 0);
		middleMoter.setSpeed(-700);
		middleMoter.backward();
		
		Delay.msDelay(1000);
		
		middleMoter.setSpeed(0);
		middleMoter.backward();
	}
	
	// �͂Ȃ�
	private static void releaseBlock() {
		
		motor_set(0, 0);
		middleMoter.setSpeed(700);
		middleMoter.forward();
		
		Delay.msDelay(1000);
		
		middleMoter.setSpeed(0);
		middleMoter.forward();
	}
	
	// ������܂őO�i
	private static float forwardFindBlock(int motor_pow) {
		
		// ���[�^�[�p�x������
		motor_init();
		
		//�W���C���Z���T�[�̃��Z�b�g
		gyroSensor.reset();
		// �Z���T�[�l��z��Ɋi�[
		gyro.fetchSample(valueGyro, 0);
		
		motor_set(motor_pow, motor_pow);
		
		int val = 5;
		//if (motor_pow < 0) val *= -1;
		
		//  �u���b�N��������܂őO�i
		while(!Button.ESCAPE.isDown() && !isFindBlock()){
			// �Z���T�[�l��z��Ɋi�[
			gyro.fetchSample(valueGyro, 0);
			if (valueGyro[0] > 0) motor_set(motor_pow + val, motor_pow - val);
			else motor_set(motor_pow - val, motor_pow + val);
			Delay.msDelay(1);
		}
		
		return (Math.abs(rightMoter.getTachoCount())+Math.abs(leftMoter.getTachoCount()))*0.5f;
	}
	
	// ������܂őO�i
	private static float forwardFindBlockPID(int motor_pow, int angle) {

		// ���[�^�[�p�x������
		motor_init();
		
		//�W���C���Z���T�[�̃��Z�b�g
		gyroSensor.reset();
		
		motor_set(motor_pow, motor_pow);
		
		lineTracePIDInit();
		
		//�Z���T�[�l��p�ӂ����z��Ɋi�[
		color.fetchSample(valueCol, 0);
		
		//  �u���b�N��������܂őO�i
		while(!Button.ESCAPE.isDown() && !isFindBlock()){

			//�Z���T�[�l��p�ӂ����z��Ɋi�[
			color.fetchSample(valueCol, 0);
			
			float nowColorVal = valueCol[0];
			
			/*
			//�Z���T�[�lLCD�\������
			for(int k=0; k<color.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueCol[k] + "m", 1, k+1);
			}*/
			
			lineTracePID(motor_pow, nowColorVal, BLACK_VALUE, angle);
		}
		
		return (Math.abs(rightMoter.getTachoCount())+Math.abs(leftMoter.getTachoCount()))*0.5f;
	}
	
	private static void angle(int pow, int ang, int angle) {
		// ���[�^�[�p�x������
		motor_init();
		
		//�W���C���Z���T�[�̃��Z�b�g
		gyroSensor.reset();
		
		motor_set(angle * pow, -angle * pow);
		
		while(true){
			
			// �Z���T�[�l��z��Ɋi�[
			gyro.fetchSample(valueGyro, 0);
			if (Math.abs(valueGyro[0]) > ang) break;
			Delay.msDelay(1);
		}
		
		motor_set(0, 0);
		Delay.msDelay(1000);
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
		
		float p = 30.0f * diff;
		float i = 20.0f * integral;
		float d = 1.0f * (diff - preDiff) / DELTA_T;
		
		float delta = (p + i + d);
		float clip = 10.0f;
		if (delta > clip) delta = clip;
		if (delta < -clip) delta = -clip;
		motor_set((int)(speed - ang * delta), (int)(speed + ang * delta));
		Delay.msDelay(10);
		LCD.refresh();
	}
	
	
	
	// PID������
	public static void lineTracePIDInit(){
		// �J���[�Z���T�[�̑O�̕΍�
		preDiff = 0;
		diff = 0;
		// I�̐ϕ��p
		integralList = new ArrayList<Float>();
	}
	
	
}
