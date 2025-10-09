/*
 * describe: Attitude calculate
 * 濠殿喗瀵ч敓鎴掓祰琚欑紒鐘愁殜閸庢挳宕氶敓锟? * @description: 闁告瑯浜〒鍓佹啺娴ｇ懓惟闂傚嫸鎷烽柧鏄忔〃閸楀酣鎯冮崟顒佹闁硅鍠曠槐鍫曞礂閵夈儱绁柛娆樺灲缁辨繈鎮ч悷鏉跨劶婵炲鍔嶉崜鎵喆閹烘鎷烽悢宄邦唺 闁告梻濞€閿熺晫鍠庣€规娊宕￠弴姘辩Т閻熸洑鐒﹀畷鑼不濡ゅ纾奸柨娑虫嫹
 * Attitude_Calculate闁哄嫷鍨板崗闁诡兛娴囪缂佺姵顨呴崵閬嶅极閿燂拷 闁告瑯鍨禍鎺旀導濞戞ɑ鍎曢柣婊庡灟缁″啴宕ｉ娆庣鞍濠靛鍋橀懙鎴﹀棘閿燂拷 闁挎稑鐡憈titude_Init闂傚嫸鎷烽柧鏄忔〃閸楀孩顦板Ο鍝勭仴濠殿喖顑呯€碉拷
 * 閺夆晜鐟ら柌婊勭閿濆洨鍨抽柡鍕靛灣缁増寰勮椤斿秹鎯冮崟顐嶎參骞€娴ｅ彨鎺旂不濡ゅ啯鐣盌EMO 闁糕晞妗ㄧ花鐞瞞u963ra闁挎稑鑻々褔寮稿鍕﹀ù锝堟硶閺侊拷660ra濞寸姴鎳樺〒鍓佹啺娴ｇ懓惟963闁轰焦婢橀悺褔宕楅妸鈺佸姤闁瑰箍鍨洪崹锟?60閻忓繗绮鹃、锟? * 濞撴艾顑呴々锟?gyroscope[0]= imu963ra_gyro_transition(imu963ra_gyro_x)* 0.0174533f; 闁告瑯浜ｉ々锕傚箮閿燂拷963闁轰焦婢橀悺褔寮ㄧ憴鍕亣660ra闁告鍟胯ぐ锟? */
#include "Attitude.h"
#include "zf_common_headfile.h"
#include "QuaternionEKF.h"
#pragma section all "cpu0_dsram"
//==================================================================================================
#define gyroscope_threshold 5

// 娣诲姞缂哄け鐨勭粨鏋勪綋瀹氫箟
typedef struct
{
    float pitch;
    float roll;
    float yaw;
    float roll_offset;
} EulerAngle_t;

typedef struct
{
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;
    int16 acc_x;
    int16 acc_y;
    int16 acc_z;
} IcmData_t;

EulerAngle_t eulerAngle = {0};
IcmData_t icm_data = {0};

//--------------------------------------------------------------------------------------
float gyroscopeOffset[3] = {0.0f, 0.0f, 0.0f}; // gyro
float gyroscope[3] = {0.0f, 0.0f, 0.0f};       // gyro
float accelerometer[3] = {0.0f, 0.0f, 0.0f};   // acc
// float
//--------------------------------------------------------------------------------------
//===================================================================================================

void Attitude_Init(void)
{
    IMU_QuaternionEKF_Init(100, 0.00001, 100000000, 0.9996, 0.001f, 0); // ekf闁告帗绻傞～鎰板礌閿燂拷
    //    lpf_init(&lpf_roll,0.2 );
    eulerAngle.roll_offset = 0;
    //===========================================================================================
    //    for (int i = 0; i < 1000; ++i) { //0婵犳洖鍊搁崹鍨叏鐎ｎ亜顕?    ////        imu963ra_get_gyro();
    //        imu963ra_get_gyro();
    //        //0婵犳洖鍊搁崹鍨叏鐎ｎ亜顕ч梻鍐ㄧ墕閿熺晫鍘ч崹浠嬪棘閿燂拷
    //        if(fabsf(gyroscope[0])+ fabsf(gyroscope[1])+ fabsf(gyroscope[2])<gyroscope_threshold) {
    //            gyroscopeOffset[0] += imu963ra_gyro_transition(imu963ra_gyro_x)* 0.0174533f;
    //            gyroscopeOffset[1] += imu963ra_gyro_transition(imu963ra_gyro_y)* 0.0174533f;
    //            gyroscopeOffset[2] += imu963ra_gyro_transition(imu963ra_gyro_z)* 0.0174533f;
    //        }
    //        else
    //        {
    //            i--;   //閻℃帒鎳撶换鍐⒓閸繐鎷烽梻瀵稿晩闁告劕绉磋ぐ鍥ㄧ▔閿熻棄鈻庨敓锟?    //        }
    //        system_delay_ms(1);  //闁圭顦遍崣搴ㄦ嚊椤忓嫮绠掗柣銊ュ閸ｄ即寮芥搴¤姵闁哄鍎辫ぐ鍥磹閿燂拷 demo闁哄嫸鎷?000Hz闁告鎷?ms濞戞搩鍘介弻锟?    //    }
    //    gyroscopeOffset[0]/=1000;
    //    gyroscopeOffset[1]/=1000;
    //    gyroscopeOffset[2]/=1000;
    gyroscopeOffset[0] = -2;
    gyroscopeOffset[1] = -10;
    gyroscopeOffset[2] = 2;
    imu660rb_get_acc();
    imu660rb_get_gyro();
    //=============================================================================================
    gyroscope[0] = imu660rb_gyro_transition(imu660rb_gyro_x - gyroscopeOffset[0]) * 0.0174533f;
    gyroscope[1] = imu660rb_gyro_transition(imu660rb_gyro_y - gyroscopeOffset[1]) * 0.0174533f;
    gyroscope[2] = imu660rb_gyro_transition(imu660rb_gyro_z - gyroscopeOffset[2]) * 0.0174533f;
    accelerometer[0] = imu660rb_acc_transition(imu660rb_acc_x);
    accelerometer[1] = imu660rb_acc_transition(imu660rb_acc_y);
    accelerometer[2] = imu660rb_acc_transition(imu660rb_acc_z);
}

#define cheat_define 0.008 // 0.0008
void Attitude_Calculate(void)
{
    // replace this with actual gyroscope data in degrees/s
    imu660rb_get_acc();
    imu660rb_get_gyro();
    gyroscope[0] = imu660rb_gyro_transition(imu660rb_gyro_x - gyroscopeOffset[0]) * 0.0174533f;
    gyroscope[1] = imu660rb_gyro_transition(imu660rb_gyro_y - gyroscopeOffset[1]) * 0.0174533f;
    gyroscope[2] = imu660rb_gyro_transition(imu660rb_gyro_z - gyroscopeOffset[2]) * 0.0174533f;
    accelerometer[0] = imu660rb_acc_transition(imu660rb_acc_x);
    accelerometer[1] = imu660rb_acc_transition(imu660rb_acc_y);
    accelerometer[2] = imu660rb_acc_transition(imu660rb_acc_z);
    // imu963ra_get_mag();
    // 閻熸洑娴囩换妯兼偘瀹€鍐炬健閹艰揪绠戦崹蹇涘箲閿燂拷 闁稿繗娓圭紞瀣喆閹烘垵顔婇柛妤佹磻缂嶅懐鎷犳搴㈢畽濞戞挸顑夊浼存儍閸曨剚鏆堥梺璇ф嫹
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
    if (fabsf(gyroscope[0]) < cheat_define)
        gyroscope[0] = 0;
    if (fabsf(gyroscope[1]) < cheat_define)
        gyroscope[1] = 0;
    if (fabsf(gyroscope[2]) < cheat_define)
        gyroscope[2] = 0;

    //===================================================================================================
    IMU_QuaternionEKF_Update(gyroscope[0], gyroscope[1], gyroscope[2], accelerometer[0], accelerometer[1], accelerometer[2]);
    //    printf("%f,%f,%f\n",  QEKF_INS.Roll,QEKF_INS.Gyro[0],QEKF_INS.Gyro[2]);

    eulerAngle.pitch = -QEKF_INS.Pitch;
    eulerAngle.yaw = QEKF_INS.Yaw;
    eulerAngle.roll = -(-eulerAngle.roll_offset + QEKF_INS.Roll);
    //    LowPassFilter_operator(&lpf_roll,eulerAngle.roll);

    icm_data.gyro_x = QEKF_INS.Gyro[0];
    icm_data.gyro_y = QEKF_INS.Gyro[1];
    icm_data.gyro_z = QEKF_INS.Gyro[2];

    icm_data.acc_x = QEKF_INS.Accel[0];
    icm_data.acc_y = QEKF_INS.Accel[1];
    icm_data.acc_z = QEKF_INS.Accel[2];
}
#pragma section all restore
