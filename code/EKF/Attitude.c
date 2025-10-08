/*
 * describe: Attitude calculate
 * 婵寧锟戒浇袙缁犳鍎撮崚锟�
 * @description: 閸欘亪娓剁憰浣瑰Ω闂勶拷閾昏桨鍗庨惃鍕殶閹诡喕绱堕崗銉ュ祮閸欘垽绱濋悧鐟板焼濞夈劍鍓扮憴鎺楋拷鐔峰 閸旂娀锟界喎瀹抽崡鏇氱秴鐟曚焦宕茬粻妤嬬磼閿涳拷
 * Attitude_Calculate閺勵垰协閹浇袙缁犳鍤遍弫锟� 閸欘垯浜掔挧娑樻儕閻滎垯绡冮崣顖欎簰婵夌偘鑵戦弬锟� 閿涘瓑ttitude_Init闂勶拷閾昏桨鍗庢妯哄灥婵瀵�
 * 鏉╂瑤閲滄禒锝囩垳閺勵垳绮版径褍顔嶉惃鍕幀浣叫掔粻妤冩畱DEMO 閸╄桨绨琲mu963ra閿涘苯顩ч弸婊勬Ц娴ｈ法鏁�660ra娴犲懘娓剁憰浣瑰Ω963閺佹澘鐡ч崗銊╁劥閹广垺鍨�660鐏忚精顢�
 * 娓氬顩� gyroscope[0]= imu963ra_gyro_transition(imu963ra_gyro_x)* 0.0174533f; 閸欘亣顩﹂幎锟�963閺佹澘鐡ч弨瑙勫灇660ra閸楀啿褰�
 */
#include "Attitude.h"
#include "zf_common_headfile.h"
#include "QuaternionEKF.h"
#pragma section all "cpu0_dsram"
//==================================================================================================
#define gyroscope_threshold 5
//--------------------------------------------------------------------------------------
float gyroscopeOffset[3] = {0.0f, 0.0f, 0.0f};    //gyro
float gyroscope[3] ={0.0f,0.0f,0.0f};                   //gyro
float accelerometer[3] = {0.0f, 0.0f, 0.0f};    //acc
//float
//--------------------------------------------------------------------------------------
//===================================================================================================

void Attitude_Init(void)
{
    IMU_QuaternionEKF_Init(100, 0.00001, 100000000, 0.9996, 0.001f,0); //ekf閸掓繂顫愰崠锟�
//    lpf_init(&lpf_roll,0.2 );
    eulerAngle.roll_offset = 0;
    //===========================================================================================
//    for (int i = 0; i < 1000; ++i) { //0濠曞倸鍨垫慨瀣
////        imu963ra_get_gyro();
//        imu963ra_get_gyro();
//        //0濠曞倸鍨垫慨瀣闂冨牆锟界厧鍨介弬锟�
//        if(fabsf(gyroscope[0])+ fabsf(gyroscope[1])+ fabsf(gyroscope[2])<gyroscope_threshold) {
//            gyroscopeOffset[0] += imu963ra_gyro_transition(imu963ra_gyro_x)* 0.0174533f;
//            gyroscopeOffset[1] += imu963ra_gyro_transition(imu963ra_gyro_y)* 0.0174533f;
//            gyroscopeOffset[2] += imu963ra_gyro_transition(imu963ra_gyro_z)* 0.0174533f;
//        }
//        else
//        {
//            i--;   //鐡掑懓绻冮梼鍫濓拷闂寸啊閸愬秴褰囨稉锟藉▎锟�
//        }
//        system_delay_ms(1);  //閹稿鍙庨懛顏勭箒閻ㄥ嫰鍣伴弽椋庡芳閺夈儱褰囬崐锟� demo閺勶拷1000Hz閸楋拷1ms娑擃厽鏌�
//    }
//    gyroscopeOffset[0]/=1000;
//    gyroscopeOffset[1]/=1000;
//    gyroscopeOffset[2]/=1000;
    gyroscopeOffset[0]=-2;
    gyroscopeOffset[1]=-10;
    gyroscopeOffset[2]=2;
    imu660rb_get_acc();
    imu660rb_get_gyro();
    //=============================================================================================
    gyroscope[0]= imu660rb_gyro_transition(gyro_gyro_x-gyroscopeOffset[0])* 0.0174533f;
    gyroscope[1]= imu660rb_gyro_transition(gyro_gyro_y-gyroscopeOffset[1])* 0.0174533f;
    gyroscope[2]= imu660rb_gyro_transition(gyro_gyro_z-gyroscopeOffset[2])* 0.0174533f;
    accelerometer[0]=imu660rb_acc_transition(gyro_acc_x);
    accelerometer[1]=imu660rb_acc_transition(gyro_acc_y);
    accelerometer[2]=imu660rb_acc_transition(gyro_acc_z);
}

#define cheat_define 0.008 //0.0008
void Attitude_Calculate(void)
{
    // replace this with actual gyroscope data in degrees/s
    imu660rb_get_acc();
    imu660rb_get_gyro();
    gyroscope[0]= imu660rb_gyro_transition(gyro_gyro_x-gyroscopeOffset[0])* 0.0174533f;
    gyroscope[1]= imu660rb_gyro_transition(gyro_gyro_y-gyroscopeOffset[1])* 0.0174533f;
    gyroscope[2]= imu660rb_gyro_transition(gyro_gyro_z-gyroscopeOffset[2])* 0.0174533f;
    accelerometer[0]=imu660rb_acc_transition(gyro_acc_x);
    accelerometer[1]=imu660rb_acc_transition(gyro_acc_y);
    accelerometer[2]=imu660rb_acc_transition(gyro_acc_z);
    //imu963ra_get_mag();
    //鐟曚浇绻樼悰宀冾潡鎼达箑鍨忛幑锟� 閸忚渹缍嬬憴鎺戝閸楁洑缍呯拠椋庢箙娑撳娼伴惃鍕暈闁诧拷
//    int repeat_flag=1;
//    float temp_gyroscope[3] ={0.0f,0.0f,0.0f};                   //gyro
//    float temp_accelerometer[3] = {0.0f, 0.0f, 0.0f};    //acc
//    while (repeat_flag == 1)
//    {
//        imu963ra_get_acc();
//        imu963ra_get_gyro();
//
//        temp_gyroscope[0]= imu963ra_gyro_transition(gyro_gyro_x-gyroscopeOffset[0])* 0.0174533f;
//        if(fabs(fabs(temp_gyroscope[0])-fabs(gyroscope[0]))>10) continue;
//        gyroscope[0] = temp_gyroscope[0];
//
//        temp_gyroscope[1]= imu963ra_gyro_transition(gyro_gyro_y-gyroscopeOffset[1])* 0.0174533f;
//        if(fabs(fabs(temp_gyroscope[1])-fabs(gyroscope[1]))>10) continue;
//        gyroscope[1] = temp_gyroscope[1];
//
//        temp_gyroscope[2]= imu963ra_gyro_transition(gyro_gyro_z-gyroscopeOffset[2])* 0.0174533f;
//        if(fabs(fabs(temp_gyroscope[2])-fabs(gyroscope[2]))>10) continue;
//        gyroscope[2] = temp_gyroscope[2];
//
//        temp_accelerometer[0]=imu963ra_acc_transition(gyro_acc_x);
//        if(fabs(fabs(temp_accelerometer[0])-fabs(accelerometer[0]))>10) continue;
//        accelerometer[0] = temp_accelerometer[0];
//
//        temp_accelerometer[1]=imu963ra_acc_transition(gyro_acc_y);
//        if(fabs(fabs(temp_accelerometer[1])-fabs(accelerometer[1]))>10) continue;
//        accelerometer[1] = temp_accelerometer[1];
//
//        temp_accelerometer[2]=imu963ra_acc_transition(gyro_acc_z);
//        if(fabs(fabs(temp_accelerometer[2])-fabs(accelerometer[2]))>10) continue;
//        accelerometer[2] = temp_accelerometer[2];
//
//
//        repeat_flag=0;
//    }

    //=================================================================
//    gyroscope[0]-= gyroscopeOffset[0];
//    gyroscope[1]-= gyroscopeOffset[1];
//    gyroscope[2]-= gyroscopeOffset[2];


    //===============================================================================
    if(fabsf(gyroscope[0])<cheat_define) gyroscope[0]=0;
    if(fabsf(gyroscope[1])<cheat_define) gyroscope[1]=0;
    if(fabsf(gyroscope[2])<cheat_define) gyroscope[2]=0;

//===================================================================================================
    IMU_QuaternionEKF_Update(gyroscope[0],gyroscope[1],gyroscope[2],accelerometer[0],accelerometer[1],accelerometer[2]);
//    printf("%f,%f,%f\n",  QEKF_INS.Roll,QEKF_INS.Gyro[0],QEKF_INS.Gyro[2]);


    eulerAngle.pitch = -QEKF_INS.Pitch;
    eulerAngle.yaw = QEKF_INS.Yaw;
    eulerAngle.roll =-(-eulerAngle.roll_offset+ QEKF_INS.Roll);
//    LowPassFilter_operator(&lpf_roll,eulerAngle.roll);

    icm_data.gyro_x= QEKF_INS.Gyro[0];
    icm_data.gyro_y = QEKF_INS.Gyro[1];
    icm_data.gyro_z = QEKF_INS.Gyro[2];

    icm_data.acc_x =QEKF_INS.Accel[0];
    icm_data.acc_y =QEKF_INS.Accel[1];
    icm_data.acc_z =QEKF_INS.Accel[2];
}
#pragma section all restore
