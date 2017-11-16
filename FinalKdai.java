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
	
	// 使用するモーターの定義
	static RegulatedMotor leftMoter = Motor.C;
	static RegulatedMotor rightMoter = Motor.B;
	static RegulatedMotor middleMoter = Motor.D;

	//使用するセンサーの定義
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
	static EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
	
	//センサーモードの設定
	static SensorMode gyro;
	static SensorMode color;
	static float valueCol[];
	
	static float[] valueGyro;
	
	static SensorMode sonic;
	static float[] valueSonic;
	
	// カラーセンサーの前の偏差
	static float preDiff = 0;
	static float diff = 0;
	
	static float DELTA_T = 0.005f;
	
	// 黒のあたい
	static float BLACK_VALUE = 0.15f;
	static float WHITE_VALUE = 0.8f;
	static float BLUE_VALUE = 0.05f;
	
	// Iの積分用
	static List<Float> integralList = new ArrayList<Float>();
	
	// 障害物の閾値
	public static final float THRESHOLD = 0.06f;
	
	public static int CLOCKWISE = 1;
	public static int COUNTERCLOCKWISE = -1;
	
	public static void main(String[] args) {
		
		//センサーモードの設定
		color = colorSensor.getMode(1);
		//センサーの取得値を格納する配列の用意
		valueCol = new float[color.sampleSize()];
		
		//センサーモードの設定
		sonic = sonicSensor.getMode(0);
		//センサーの取得値を格納する配列の用意
		valueSonic = new float[sonic.sampleSize()];
		
		//センサーモードの設定
		gyro = gyroSensor.getMode(1);
		//センサーの取得値を格納する配列の用意
		valueGyro = new float[gyro.sampleSize()];
		//ジャイロセンサーのリセット
		gyroSensor.reset();
		
		//画像の初期化
		LCD.clear();
		
		// モーター角度初期化
		motor_init();
		
		int pow = 100;
		int powHigh = 150;
		
		// 前進、PID
		forwardPID(pow/2, 300, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// モーター初期化
		motor_init();
		
		// 前進
		forward(powHigh+4, powHigh, 360);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		//  ブロックが見つかるまで前進
		float deg = forwardFindBlockPID(pow, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// ブロックをつかむ
		catchBlock();
		
		// 後進
		forward(-pow/2, deg + 30);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// 回転
		angle(pow/2, 90, CLOCKWISE);
		
		// 前進
		forward(powHigh, powHigh+4, 290);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// 前進、PID
		forwardPID(pow * 2, 1470, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// 前進
		forward(powHigh, 475);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// 回転、誤差あり？
		angle(pow/2, 78, COUNTERCLOCKWISE);
		
		// 前進
		forward(powHigh, 100);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// 前進、PID
		forwardPID(pow/2, 320, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// はなす
		releaseBlock();
		
		// 後進
		forward(-pow/2, 200);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		angle(pow/2, 170, COUNTERCLOCKWISE);
		
		forward(powHigh, 300);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		//  ブロックが見つかるまで前進
		deg = forwardFindBlockPID(pow/2, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
			
		// ブロックをつかむ
		catchBlock();
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// 後進　
		forward(-pow+10, -pow, deg + 200);
		//forward(-pow, deg + 200);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		angle(pow/2, 172, CLOCKWISE);
		
		// 前進、PID
		forwardPID(pow/3, 320, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// はなす
		releaseBlock();
		
		motor_set(0, 0);
		Delay.msDelay(2000);
		
		// 前進
		forward(pow, 100);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// 回転
		angle(pow/2, 170, COUNTERCLOCKWISE);
		
		//  ブロックが見つかるまで前進
		deg = forwardFindBlockPID(pow/2, CLOCKWISE);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// ブロックをつかむ
		catchBlock();
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// 回転
		angle(pow/2, 170, CLOCKWISE);
		
		// はなす
		releaseBlock();
		
		motor_set(0, 0);
		Delay.msDelay(1000);
		
		// 前進
		forward(-pow, 100);
		
		motor_set(0, 0);
		Delay.msDelay(1000);
	}
	
	/**
	 * モーター初期化
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
	
	// deg度全身
	private static void forward(int motor_pow, float deg) {
		
		// モーター角度初期化
		motor_init();
		
		// ジャイロセンサーのリセット
		gyroSensor.reset();
		
		motor_set(motor_pow, motor_pow);
		
		int val = 3;
		//if (motor_pow < 0) val *= -1;
		
		while((Math.abs(rightMoter.getTachoCount())+Math.abs(leftMoter.getTachoCount()))*0.5f < deg){
			
			// センサー値を配列に格納
			gyro.fetchSample(valueGyro, 0);
			if (valueGyro[0] > 0) motor_set(motor_pow + val, motor_pow - val);
			else motor_set(motor_pow - val, motor_pow + val);
			Delay.msDelay(1);
		}
	}
	
	// deg度全身
	private static void forward(int l_motor_pow, int r_motor_pow, float deg) {
		
		// モーター角度初期化
		motor_init();
		
		// ジャイロセンサーのリセット
		gyroSensor.reset();
		
		motor_set(l_motor_pow, r_motor_pow);
		
		int val = 3;
		//if (motor_pow < 0) val *= -1;
		
		while((Math.abs(rightMoter.getTachoCount())+Math.abs(leftMoter.getTachoCount()))*0.5f < deg){
			
			// センサー値を配列に格納
			gyro.fetchSample(valueGyro, 0);
			if (valueGyro[0] > 0) motor_set(l_motor_pow + val, r_motor_pow - val);
			else motor_set(l_motor_pow - val, r_motor_pow + val);
			Delay.msDelay(1);
		}
	}
	
	// deg度全身
	private static void forwardPID(int motor_pow, float deg, int angle) {
		
		// モーター角度初期化
		motor_init();
		
		//ジャイロセンサーのリセット
		gyroSensor.reset();
		
		motor_set(motor_pow, motor_pow);
		
		lineTracePIDInit();
		
		//センサー値を用意した配列に格納
		color.fetchSample(valueCol, 0);
		
		while((Math.abs(rightMoter.getTachoCount())+Math.abs(leftMoter.getTachoCount()))*0.5f < deg){
			
			//センサー値を用意した配列に格納
			color.fetchSample(valueCol, 0);
			
			float nowColorVal = valueCol[0];
			/*
			//センサー値LCD表示部分
			for(int k=0; k<color.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueCol[k] + "m", 1, k+1);
			}*/
			
			lineTracePID(motor_pow, nowColorVal, BLACK_VALUE, angle);
		}
	}
	
	// ブロックを見つけたかどうか
	private static boolean isFindBlock() {
		
		//センサー値を配列に格納
		sonic.fetchSample(valueSonic, 0);
	    if (valueSonic[0] <= THRESHOLD) return true;
		
		return false;
	}
	
	// つかむ
	private static void catchBlock() {
		
		// モーター角度初期化
		motor_init();
		motor_set(0, 0);
		middleMoter.setSpeed(-700);
		middleMoter.backward();
		
		Delay.msDelay(1000);
		
		middleMoter.setSpeed(0);
		middleMoter.backward();
	}
	
	// はなす
	private static void releaseBlock() {
		
		motor_set(0, 0);
		middleMoter.setSpeed(700);
		middleMoter.forward();
		
		Delay.msDelay(1000);
		
		middleMoter.setSpeed(0);
		middleMoter.forward();
	}
	
	// 見つけるまで前進
	private static float forwardFindBlock(int motor_pow) {
		
		// モーター角度初期化
		motor_init();
		
		//ジャイロセンサーのリセット
		gyroSensor.reset();
		// センサー値を配列に格納
		gyro.fetchSample(valueGyro, 0);
		
		motor_set(motor_pow, motor_pow);
		
		int val = 5;
		//if (motor_pow < 0) val *= -1;
		
		//  ブロックが見つかるまで前進
		while(!Button.ESCAPE.isDown() && !isFindBlock()){
			// センサー値を配列に格納
			gyro.fetchSample(valueGyro, 0);
			if (valueGyro[0] > 0) motor_set(motor_pow + val, motor_pow - val);
			else motor_set(motor_pow - val, motor_pow + val);
			Delay.msDelay(1);
		}
		
		return (Math.abs(rightMoter.getTachoCount())+Math.abs(leftMoter.getTachoCount()))*0.5f;
	}
	
	// 見つけるまで前進
	private static float forwardFindBlockPID(int motor_pow, int angle) {

		// モーター角度初期化
		motor_init();
		
		//ジャイロセンサーのリセット
		gyroSensor.reset();
		
		motor_set(motor_pow, motor_pow);
		
		lineTracePIDInit();
		
		//センサー値を用意した配列に格納
		color.fetchSample(valueCol, 0);
		
		//  ブロックが見つかるまで前進
		while(!Button.ESCAPE.isDown() && !isFindBlock()){

			//センサー値を用意した配列に格納
			color.fetchSample(valueCol, 0);
			
			float nowColorVal = valueCol[0];
			
			/*
			//センサー値LCD表示部分
			for(int k=0; k<color.sampleSize(); k++){
				LCD.drawString("val[" + k + "] : " + valueCol[k] + "m", 1, k+1);
			}*/
			
			lineTracePID(motor_pow, nowColorVal, BLACK_VALUE, angle);
		}
		
		return (Math.abs(rightMoter.getTachoCount())+Math.abs(leftMoter.getTachoCount()))*0.5f;
	}
	
	private static void angle(int pow, int ang, int angle) {
		// モーター角度初期化
		motor_init();
		
		//ジャイロセンサーのリセット
		gyroSensor.reset();
		
		motor_set(angle * pow, -angle * pow);
		
		while(true){
			
			// センサー値を配列に格納
			gyro.fetchSample(valueGyro, 0);
			if (Math.abs(valueGyro[0]) > ang) break;
			Delay.msDelay(1);
		}
		
		motor_set(0, 0);
		Delay.msDelay(1000);
	}
	
	private static void lineTracePID(float speed, float sensorValue, float targetValue, int ang) {
		
		float nowColorVal = sensorValue;
		
		// 偏差
		preDiff = diff;
		diff = nowColorVal - targetValue;
		
		float tmp = (diff + preDiff) / 2.0f * DELTA_T;
		integralList.add(tmp);
		
		// 10個まで
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
	
	
	
	// PID初期化
	public static void lineTracePIDInit(){
		// カラーセンサーの前の偏差
		preDiff = 0;
		diff = 0;
		// Iの積分用
		integralList = new ArrayList<Float>();
	}
	
	
}
