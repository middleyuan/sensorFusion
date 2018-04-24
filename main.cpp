#include "mbed.h"
#include "LSM9DS1.h"
#include "AS5145.h"
 
LSM9DS1 imu2(D5, D7);
LSM9DS1 imu3(D3, D6);
LSM9DS1 imu(D14, D15);
AnalogIn sEMG(D13);
AnalogIn sEMG2(D1);
AnalogIn sEMG3(D0);
AnalogIn sEMG4(PC_4);
Serial pc(USBTX, USBRX);
Ticker timer1;
Ticker timer2;
 
float T = 0.001;
/********************************************************************/
//function declaration
/********************************************************************/
void init_TIMER(void);
void timer1_interrupt(void);
void setup(void);
void estimator(float axm[3],float aym[3],float azm[3],float w3[3],float w2[3],float w1[3],float alpha);
float lpf(float input, float output_old, float frequency);
void angle_fn(float x1_hat[3],float x2_hat[3]);
void pitch_dot_fn(float w3[3],float w2[3],float w1[3],float sinroll[3],float cosroll[3]);
void pitch_double_dot_fn(float pitch_dot[3],float pitch_dot_old[3]);
/********************************************************************/
// sensor data
/********************************************************************/
int16_t Gyro_axis_data[9] = {0};     // X, Y, Z axis
int16_t Acce_axis_data[9] = {0};     // X, Y, Z axis
float Gyro_axis_data_f[9] = {0};
float Gyro_axis_data_f_old[9] = {0};
float Acce_axis_data_f[9] = {0};
float Acce_axis_data_f_old[9] = {0};
float axm[3] = {0.0f};
float aym[3] = {0.0f};
float azm[3] = {0.0f};
float w1[3] = {0.0f};
float w2[3] = {0.0f};
float w3[3] = {0.0f};
//estimator
float x1_hat[3] = {0.0f};
float x2_hat[3] = {0.0f};
float sinroll[3] = {0.0f};
float cosroll[3] = {0.0f};
float sinpitch[3] = {0.0f};
float pitch_angle[3] = {0.0f};
float roll_angle[3] = {0.0f};
float yaw_dot[3] = {0.0f};
float pitch_dot[3] = {0.0f};
float pitch_double_dot[3] = {0.0f};
float pitch_double_dot_f[3] = {0.0f};
float pitch_double_dot_f_old[3] = {0.0f};
float pitch_dot_old[3] = {0.0f};
float axm_f[3] = {0.0f};
float axm_f_old[3] = {0.0f};
float w3aym_f[3] = {0.0f};
float w3aym_f_old[3] = {0.0f};
float w2azm_f[3] = {0.0f};
float w2azm_f_old[3] = {0.0f};
float aym_f[3] = {0.0f};
float aym_f_old[3] = {0.0f};
float w3axm_f[3] = {0.0f};
float w3axm_f_old[3] = {0.0f};
float w1azm_f[3] = {0.0f};
float w1azm_f_old[3] = {0.0f};
float azm_f[3] = {0.0f};
float azm_f_old[3] = {0.0f};
float w2axm_f[3] = {0.0f};
float w2axm_f_old[3] = {0.0f};
float w1aym_f[3] = {0.0f};
float w1aym_f_old[3] = {0.0f};
//sEMG variable
float emg_value[4] = {0.0f};
 
int main()
{
    pc.baud(230400);
    setup();  //Setup sensors
    AS5145_begin(); //begin encoder
    init_TIMER();
    while (1)
    {
        //pc.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",pitch_angle[0],pitch_dot[0],pitch_angle[1],pitch_dot[1],pitch_angle[2],pitch_dot[2],emg_value[0],emg_value[1],emg_value[2],emg_value[3],position[1]*360/4096, position[0]*360/4096);
        wait(0.05);
        //pc.printf("%f,%f,%f,%f\n",emg_value[0],emg_value[1],emg_value[2],emg_value[3]);
        pc.printf("%f,%f,%f,%f,%f,%d,%d\n", pitch_angle[0], pitch_angle[1], roll_angle[1], pitch_angle[2], roll_angle[2], position[1]*360/4096, position[0]*360/4096);
        //pc.printf("IMU: %2f,%2f\r\n", pitch_angle[0], roll_angle[0]);
        //pc.printf("IMU2: %2f,%2f\r\n", pitch_angle[1], roll_angle[1]);
        //pc.printf("IMU3: %2f,%2f\r\n", pitch_angle[2], roll_angle[2]);
        //pc.printf("position: %d,%d\r\n", position[0], position[1]);
        //pc.printf("roll_angle: %2f\r\n",roll_angle);
        //pc.printf("A: %2f, %2f, %2f\r\n", imu2.ax*Acce_gain_x_2, imu2.ay*Acce_gain_y_2, imu2.az*Acce_gain_z_2);
    }
}
void setup()
{
    imu.begin();
    imu2.begin();
    imu3.begin();
}
/********************************************************************/
// init_TIMER
/********************************************************************/
void init_TIMER(void)
{
    timer1.attach_us(&timer1_interrupt, 10000);//10ms interrupt period (100 Hz)
    timer2.attach_us(&ReadValue, 1000);//1ms interrupt period (1000 Hz)
}
/********************************************************************/
// timer1_interrupt
/********************************************************************/
void timer1_interrupt(void)
{  
    int i;
    imu.readAccel();
    imu.readGyro();
    imu2.readAccel();
    imu2.readGyro();
    imu3.readAccel();
    imu3.readGyro();
    // sensor raw data
    Acce_axis_data[0] = imu.ax*Acce_gain_x;
    Acce_axis_data[1] = imu.ay*Acce_gain_y;
    Acce_axis_data[2] = imu.az*Acce_gain_z;
    Acce_axis_data[3] = -imu2.ax*Acce_gain_x_2;
    Acce_axis_data[4] = imu2.az*Acce_gain_y_2;
    Acce_axis_data[5] = imu2.ay*Acce_gain_z_2;
    Acce_axis_data[6] = -imu3.ax*Acce_gain_x_2;
    Acce_axis_data[7] = -imu3.az*Acce_gain_y_2;
    Acce_axis_data[8] = -imu3.ay*Acce_gain_z_2;
 
    Gyro_axis_data[0] = imu.gx*Gyro_gain_x;
    Gyro_axis_data[1] = imu.gy*Gyro_gain_y;
    Gyro_axis_data[2] = imu.gz*Gyro_gain_z;
    Gyro_axis_data[3] = -imu2.gx*Gyro_gain_x_2;
    Gyro_axis_data[4] = imu2.gz*Gyro_gain_y_2;
    Gyro_axis_data[5] = imu2.gy*Gyro_gain_z_2;
    Gyro_axis_data[6] = -imu3.gx*Gyro_gain_x_2;
    Gyro_axis_data[7] = -imu3.gz*Gyro_gain_y_2;
    Gyro_axis_data[8] = -imu3.gy*Gyro_gain_z_2;
    
    for(i=0;i<9;i++)
    {
        Acce_axis_data_f[i] = lpf(Acce_axis_data[i],Acce_axis_data_f_old[i],15);
        Acce_axis_data_f_old[i] = Acce_axis_data_f[i];
        Gyro_axis_data_f[i] = lpf(Gyro_axis_data[i],Gyro_axis_data_f_old[i],15);     
        Gyro_axis_data_f_old[i] = Gyro_axis_data_f[i];
    }
    
    axm[0] = Acce_axis_data_f[0];
    aym[0] = Acce_axis_data_f[1];
    azm[0] = Acce_axis_data_f[2];
    w1[0]  = Gyro_axis_data_f[0];
    w2[0]  = Gyro_axis_data_f[1];
    w3[0]  = Gyro_axis_data_f[2];
    axm[1] = Acce_axis_data_f[3];
    aym[1] = Acce_axis_data_f[4];
    azm[1] = Acce_axis_data_f[5];
    w1[1]  = Gyro_axis_data_f[3];
    w2[1]  = Gyro_axis_data_f[4];
    w3[1]  = Gyro_axis_data_f[5];
    axm[2] = Acce_axis_data_f[6];
    aym[2] = Acce_axis_data_f[7];
    azm[2] = Acce_axis_data_f[8];
    w1[2]  = Gyro_axis_data_f[6];
    w2[2]  = Gyro_axis_data_f[7];
    w3[2]  = Gyro_axis_data_f[8];
    
    
    estimator(axm,aym,azm,w3,w2,w1,120);
    angle_fn(x1_hat,x2_hat);
    pitch_dot_fn(w3,w2,w1,sinroll,cosroll);
    pitch_double_dot_fn(pitch_dot,pitch_dot_old); 
 
    for(i=0;i<3;i++)
    {
        pitch_dot_old[i] = pitch_dot[i];
    }
    emg_value[0] = sEMG.read(); 
    emg_value[1] = sEMG2.read(); 
    emg_value[2] = sEMG3.read();
    emg_value[3] = sEMG4.read();
}
/********************************************************************/
// estimator
/********************************************************************/
void estimator(float axm[3],float aym[3],float azm[3],float w3[3],float w2[3],float w1[3],float alpha)
{
    int i;
    for(i=0;i<3;i++)
    {
        axm_f[i] = lpf(axm[i],axm_f_old[i],alpha);
        axm_f_old[i] = axm_f[i];
        w3aym_f[i] = lpf(w3[i]*aym[i],w3aym_f_old[i],alpha);
        w3aym_f_old[i] = w3aym_f[i];
        w2azm_f[i] = lpf(w2[i]*azm[i],w2azm_f_old[i],alpha);
        w2azm_f_old[i] = w2azm_f[i];
        aym_f[i] = lpf(aym[i],aym_f_old[i],alpha);
        aym_f_old[i] = aym_f[i];
        w3axm_f[i] = lpf(w3[i]*axm[i],w3axm_f_old[i],alpha);
        w3axm_f_old[i] = w3axm_f[i];
        w1azm_f[i] = lpf(w1[i]*azm[i],w1azm_f_old[i],alpha);
        w1azm_f_old[i] = w1azm_f[i];
    
        x1_hat[i] = axm_f[i] + w3aym_f[i]/alpha - w2azm_f[i]/alpha;
        x2_hat[i] = -w3axm_f[i]/alpha + aym_f[i] + w1azm_f[i]/alpha;
    }
    
}
/********************************************************************/
// angle_fn
/********************************************************************/
void angle_fn(float x1_hat[3],float x2_hat[3])
{
    int i;
    for(i=0;i<3;i++)
    {
        sinroll[i] = x2_hat[i]*(-0.1020);
        if(sinroll[i] >= 1.0f)
        {
            sinroll[i] = 1.0;
            cosroll[i] = 0.0;
        }
        else if(sinroll[i] <= -1.0f)
        {
            sinroll[i] = -1.0;
            cosroll[i] = 0.0;
        }
        else cosroll[i] = sqrt(1-(sinroll[i]*sinroll[i]));
        roll_angle[i] = (asin(sinroll[i]))*180/pi;
        sinpitch[i] = x1_hat[i]*(0.1020f)/cosroll[i];
        if(sinpitch[i] >= 1.0f)
        {
            sinpitch[i] = 1.0;
        }
        else if(sinpitch[i] <= -1.0f)
        {
            sinpitch[i] = -1.0;
        }
    
        pitch_angle[i] = (asin(sinpitch[i]))*180/pi;
    }
}
void pitch_dot_fn(float w3[3],float w2[3],float w1[3],float sinroll[3],float cosroll[3])
{
    int i;
    for(i=0;i<3;i++)
    {
        yaw_dot[i] = (w3[i]*cosroll[i] - w1[i]*sinroll[i])/cosroll[i];
        pitch_dot[i] = w2[i] - yaw_dot[i]*sinroll[i];
    }
}
void pitch_double_dot_fn(float pitch_dot[3],float pitch_dot_old[3])
{
    int i;
    for(i=0;i<3;i++)
    {
        pitch_double_dot[i] = (pitch_dot[i] - pitch_dot_old[i])/0.01f;
        pitch_double_dot_f[i] = lpf(pitch_double_dot[i],pitch_double_dot_f_old[i],30);
        pitch_double_dot_f_old[i] = pitch_double_dot_f[i]; 
    }
}
/********************************************************************/
// lpf
/********************************************************************/
float lpf(float input, float output_old, float frequency)
{
    float output = 0;
    
    output = (output_old + frequency*T*input) / (1 + frequency*T);
    
    return output;
}
