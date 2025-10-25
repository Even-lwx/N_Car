/*********************************************************************************************************************
 * 文件名称: imu_calibration_improved.c
 * 功能说明: 改进版加速度计校准算法实现
 * 作    者: N_Car项目组
 * 日    期: 2025-01-25
 * 备    注: 核心改进：静止检测 + 高频采样 + 方向覆盖检测
 ********************************************************************************************************************/

#include "imu_calibration_improved.h"
#include "imu.h"
#include "zf_common_headfile.h"
#include "zf_device_imu660rb.h"
#include "EKF/matrix.h"  // ARM CMSIS DSP矩阵运算库
#include <math.h>

// *************************** 全局变量定义 ***************************

acc_calibration_params_t g_acc_calib_params = {
    .bias_x = 0.0f,
    .bias_y = 0.0f,
    .bias_z = 0.0f,
    .scale_x = 1.0f,
    .scale_y = 1.0f,
    .scale_z = 1.0f,
    .calibrated = 0
};

// *************************** 手动校准状态变量 ***************************

// 全局实例定义（类型声明在头文件中）
manual_calibration_state_t g_manual_calib_state = {0};

// *************************** 静态辅助函数 ***************************

/**
 * @brief       检测当前是否处于静止状态
 * @param       ax_cur, ay_cur, az_cur  当前加速度（g）
 * @param       ax_last, ay_last, az_last  上一次加速度（g）
 * @return      uint8   1=静止, 0=运动
 */
static uint8 is_device_still(float ax_cur, float ay_cur, float az_cur,
                              float ax_last, float ay_last, float az_last)
{
    float delta_ax = fabsf(ax_cur - ax_last);
    float delta_ay = fabsf(ay_cur - ay_last);
    float delta_az = fabsf(az_cur - az_last);
    float max_delta = delta_ax > delta_ay ? delta_ax : delta_ay;
    max_delta = max_delta > delta_az ? max_delta : delta_az;

    return (max_delta < ACC_CAL_STILLNESS_THRESHOLD) ? 1 : 0;
}

/**
 * @brief       更新方向覆盖统计
 * @param       ax, ay, az  平均加速度（g）
 * @param       coverage    方向覆盖统计结构体指针
 */
static void update_direction_coverage(float ax, float ay, float az, acc_direction_coverage_t *coverage)
{
    float abs_ax = fabsf(ax);
    float abs_ay = fabsf(ay);
    float abs_az = fabsf(az);
    float max_axis = abs_ax > abs_ay ? abs_ax : abs_ay;
    max_axis = max_axis > abs_az ? max_axis : abs_az;

    // 只有当主轴接近1g时才算有效方向（容差0.3g）
    if (max_axis > (1.0f - ACC_CAL_DIRECTION_TOLERANCE) &&
        max_axis < (1.0f + ACC_CAL_DIRECTION_TOLERANCE))
    {
        if (abs_ax == max_axis)
        {
            if (ax > 0)
                coverage->covered_px = 1;
            else
                coverage->covered_nx = 1;
        }
        else if (abs_ay == max_axis)
        {
            if (ay > 0)
                coverage->covered_py = 1;
            else
                coverage->covered_ny = 1;
        }
        else if (abs_az == max_axis)
        {
            if (az > 0)
                coverage->covered_pz = 1;
            else
                coverage->covered_nz = 1;
        }
    }

    // 更新计数
    coverage->count = (coverage->covered_px ? 1 : 0) + (coverage->covered_nx ? 1 : 0) +
                      (coverage->covered_py ? 1 : 0) + (coverage->covered_ny ? 1 : 0) +
                      (coverage->covered_pz ? 1 : 0) + (coverage->covered_nz ? 1 : 0);
}

/**
 * @brief       打印方向覆盖状态
 */
static void print_direction_coverage(acc_direction_coverage_t *coverage)
{
    printf("%cX %cX %cY %cY %cZ %cZ",
           coverage->covered_px ? '+' : ' ', coverage->covered_nx ? '-' : ' ',
           coverage->covered_py ? '+' : ' ', coverage->covered_ny ? '-' : ' ',
           coverage->covered_pz ? '+' : ' ', coverage->covered_nz ? '-' : ' ');
}

// *************************** 主要功能函数 ***************************

/**
 * @brief       改进版加速度计校准函数（核心实现）
 */
uint8 imu_calibrate_acc_improved(uint16 sample_count, uint32 timeout_ms)
{
    // ========== 参数检查 ==========
    if (!imu_data.is_initialized)
    {
        printf("[ACC CAL] 错误: IMU未初始化!\r\n");
        return 0;
    }

    if (sample_count < ACC_CAL_IMPROVED_MIN_SAMPLES)
    {
        printf("[ACC CAL] 样本数过少! 最少需要 %d 个样本\r\n", ACC_CAL_IMPROVED_MIN_SAMPLES);
        sample_count = ACC_CAL_IMPROVED_MIN_SAMPLES;
    }
    if (sample_count > ACC_CAL_IMPROVED_MAX_SAMPLES)
    {
        printf("[ACC CAL] 样本数过多! 最多支持 %d 个样本\r\n", ACC_CAL_IMPROVED_MAX_SAMPLES);
        sample_count = ACC_CAL_IMPROVED_MAX_SAMPLES;
    }

    // ========== 打印校准说明 ==========
    printf("\r\n========== 加速度计校准开始 (改进版 v2.0) ==========\r\n");
    printf("【重要】必须在静止状态下采样，系统会自动检测静止\r\n\r\n");
    printf("使用方法：\r\n");
    printf("  1. 缓慢旋转IMU到6个主要方向（±X, ±Y, ±Z）\r\n");
    printf("  2. 在每个方向上静止2-3秒（等待采样提示）\r\n");
    printf("  3. 系统会自动过滤运动状态的数据\r\n\r\n");
    printf("配置参数：\r\n");
    printf("  采集目标: %d 个样本组\r\n", sample_count);
    printf("  静止阈值: %.3f g\r\n", ACC_CAL_STILLNESS_THRESHOLD);
    printf("  每组采样: %d 个样本取平均\r\n", ACC_CAL_STILLNESS_SAMPLES);
    printf("  最小覆盖: %d 个方向\r\n\r\n", ACC_CAL_MIN_COVERAGE_COUNT);

    // ========== 分配矩阵内存 ==========
    float HTH_data[36] = {0};  // H'H累加器 (6×6)
    float HTb_data[6] = {0};   // H'b累加器 (6×1)
    float HTH_inv_data[36];    // H'H逆矩阵 (6×6)
    float X_data[6];           // 解向量 (6×1)

    // ========== 采样控制变量 ==========
    uint16 collected = 0;
    uint32 start_time = system_getval_ms();
    uint32 last_still_print = 0;

    float last_ax = 0, last_ay = 0, last_az = 0;
    uint8 first_sample = 1;

    acc_direction_coverage_t coverage = {0};

    printf("准备就绪，开始采集...\r\n\r\n");

    // ========== 主采样循环 ==========
    while (collected < sample_count)
    {
        // 超时检测
        if (timeout_ms > 0 && (system_getval_ms() - start_time) > timeout_ms)
        {
            printf("\r\n[ACC CAL] 采集超时! 已采集 %d/%d\r\n", collected, sample_count);
            return 0;
        }

        // 等待数据更新
        imu_data.data_ready = false;
        uint32 wait_start = system_getval_ms();
        while (!imu_data.data_ready)
        {
            if ((system_getval_ms() - wait_start) > 100)
            {
                printf("[ACC CAL] 数据更新超时! 请检查中断\r\n");
                break;
            }
            system_delay_ms(1);
        }

        if (!imu_data.data_ready)
            continue;

        // 转换为物理单位（g）
        float ax = imu660rb_acc_transition(imu_data.acc_x);
        float ay = imu660rb_acc_transition(imu_data.acc_y);
        float az = imu660rb_acc_transition(imu_data.acc_z);

        // ========== 静止检测 ==========
        if (first_sample)
        {
            last_ax = ax;
            last_ay = ay;
            last_az = az;
            first_sample = 0;
            continue;
        }

        uint8 is_still = is_device_still(ax, ay, az, last_ax, last_ay, last_az);

        // 更新上一次数据
        last_ax = ax;
        last_ay = ay;
        last_az = az;

        // 只在静止时采样
        if (!is_still)
        {
            continue;
        }

        // 静止提示（每2秒打印一次）
        if ((system_getval_ms() - last_still_print) > 2000)
        {
            printf("[静止检测] 检测到静止，开始采样...\r\n");
            last_still_print = system_getval_ms();
        }

        // ========== 高频连续采样：在静止段内采集多个样本取平均 ==========
        float ax_sum = ax, ay_sum = ay, az_sum = az;
        uint8 valid_samples = 1;

        for (uint8 i = 1; i < ACC_CAL_STILLNESS_SAMPLES; i++)
        {
            // 等待下一个数据更新（2ms周期）
            imu_data.data_ready = false;
            wait_start = system_getval_ms();
            while (!imu_data.data_ready)
            {
                if ((system_getval_ms() - wait_start) > 10)
                    break;
                // 不使用delay，直接轮询以获得最快响应
            }

            if (!imu_data.data_ready)
                continue;

            float ax_new = imu660rb_acc_transition(imu_data.acc_x);
            float ay_new = imu660rb_acc_transition(imu_data.acc_y);
            float az_new = imu660rb_acc_transition(imu_data.acc_z);

            // 检查是否仍然静止
            if (fabsf(ax_new - ax) < ACC_CAL_STILLNESS_THRESHOLD &&
                fabsf(ay_new - ay) < ACC_CAL_STILLNESS_THRESHOLD &&
                fabsf(az_new - az) < ACC_CAL_STILLNESS_THRESHOLD)
            {
                ax_sum += ax_new;
                ay_sum += ay_new;
                az_sum += az_new;
                valid_samples++;
            }
            else
            {
                // 检测到运动，放弃本组采样
                break;
            }
        }

        // 至少采集到一半样本才有效
        if (valid_samples < (ACC_CAL_STILLNESS_SAMPLES / 2))
        {
            continue;
        }

        // 计算平均值
        float ax_avg = ax_sum / valid_samples;
        float ay_avg = ay_sum / valid_samples;
        float az_avg = az_sum / valid_samples;

        // ========== 更新方向覆盖统计 ==========
        update_direction_coverage(ax_avg, ay_avg, az_avg, &coverage);

        // ========== 增量式累加 H'H 和 H'b ==========
        // 椭球拟合方程: Ax² + By² + Cz² + Dx + Ey + F = 1
        float h[6] = {ax_avg * ax_avg, ay_avg * ay_avg, az_avg * az_avg,
                      ax_avg, ay_avg, az_avg};
        float b_val = 1.0f;

        for (uint8 i = 0; i < 6; i++)
        {
            for (uint8 j = 0; j < 6; j++)
            {
                HTH_data[i * 6 + j] += h[i] * h[j];
            }
        }

        for (uint8 i = 0; i < 6; i++)
        {
            HTb_data[i] += h[i] * b_val;
        }

        collected++;

        // 进度显示（每10%打印一次）
        if (collected % (sample_count / 10 + 1) == 0 || collected == sample_count)
        {
            printf("进度: %3d%% (%d/%d) | 方向: ",
                   collected * 100 / sample_count, collected, sample_count);
            print_direction_coverage(&coverage);
            printf(" (%d/6)\r\n", coverage.count);
        }

        // 短暂延迟，避免同一静止段重复采样
        system_delay_ms(100);
    }

    // ========== 方向覆盖度检查 ==========
    printf("\r\n数据采集完成，方向覆盖度：%d/6\r\n", coverage.count);
    if (coverage.count < ACC_CAL_MIN_COVERAGE_COUNT)
    {
        printf("[警告] 方向覆盖不足！建议至少覆盖 %d 个方向\r\n", ACC_CAL_MIN_COVERAGE_COUNT);
        printf("缺失方向: ");
        if (!coverage.covered_px) printf("+X ");
        if (!coverage.covered_nx) printf("-X ");
        if (!coverage.covered_py) printf("+Y ");
        if (!coverage.covered_ny) printf("-Y ");
        if (!coverage.covered_pz) printf("+Z ");
        if (!coverage.covered_nz) printf("-Z ");
        printf("\r\n");
    }

    printf("开始求解校准参数...\r\n");

    // ========== 求解最小二乘 X = (H'H)^(-1) * H'b ==========
    arm_matrix_instance_f32 HTH, HTH_inv, HTb, X;
    arm_mat_init_f32(&HTH, 6, 6, HTH_data);
    arm_mat_init_f32(&HTH_inv, 6, 6, HTH_inv_data);
    arm_mat_init_f32(&HTb, 6, 1, HTb_data);
    arm_mat_init_f32(&X, 6, 1, X_data);

    // 计算 (H'H)^(-1)
    if (arm_mat_inverse_f32(&HTH, &HTH_inv) != ARM_MATH_SUCCESS)
    {
        printf("[ACC CAL] 矩阵求逆失败! (数据可能不够分散)\r\n");
        printf("建议: 在更多方向上旋转IMU，确保6个面都采集到数据\r\n");
        return 0;
    }

    // 计算 X = (H'H)^(-1) * H'b
    if (arm_mat_mult_f32(&HTH_inv, &HTb, &X) != ARM_MATH_SUCCESS)
    {
        printf("[ACC CAL] 最终求解失败!\r\n");
        return 0;
    }

    // ========== 提取椭球参数并转换为校准参数 ==========
    float A = X_data[0]; // x²系数
    float B = X_data[1]; // y²系数
    float C = X_data[2]; // z²系数
    float D = X_data[3]; // x系数
    float E = X_data[4]; // y系数
    float F = X_data[5]; // z系数

    // 防止除零和负数开方
    if (A <= 1e-6f || B <= 1e-6f || C <= 1e-6f)
    {
        printf("[ACC CAL] 拟合结果异常 (系数过小)!\r\n");
        return 0;
    }

    // 计算校准参数
    g_acc_calib_params.bias_x = -D / (2.0f * A);
    g_acc_calib_params.bias_y = -E / (2.0f * B);
    g_acc_calib_params.bias_z = -F / (2.0f * C);
    g_acc_calib_params.scale_x = 1.0f / sqrtf(A);
    g_acc_calib_params.scale_y = 1.0f / sqrtf(B);
    g_acc_calib_params.scale_z = 1.0f / sqrtf(C);

    // 参数合理性检查
    if (fabsf(g_acc_calib_params.bias_x) > 2.0f ||
        fabsf(g_acc_calib_params.bias_y) > 2.0f ||
        fabsf(g_acc_calib_params.bias_z) > 2.0f ||
        g_acc_calib_params.scale_x < 0.5f || g_acc_calib_params.scale_x > 2.0f ||
        g_acc_calib_params.scale_y < 0.5f || g_acc_calib_params.scale_y > 2.0f ||
        g_acc_calib_params.scale_z < 0.5f || g_acc_calib_params.scale_z > 2.0f)
    {
        printf("[ACC CAL] 参数超出合理范围!\r\n");
        printf("Bias: [%.3f, %.3f, %.3f] (期望: ±2.0g)\r\n",
               g_acc_calib_params.bias_x, g_acc_calib_params.bias_y, g_acc_calib_params.bias_z);
        printf("Scale: [%.3f, %.3f, %.3f] (期望: 0.5~2.0)\r\n",
               g_acc_calib_params.scale_x, g_acc_calib_params.scale_y, g_acc_calib_params.scale_z);
        return 0;
    }

    // ========== 输出校准结果 ==========
    printf("\r\n========== 校准成功! ==========\r\n");
    printf("零偏 (Bias):\r\n");
    printf("  X: %+.4f g\r\n", g_acc_calib_params.bias_x);
    printf("  Y: %+.4f g\r\n", g_acc_calib_params.bias_y);
    printf("  Z: %+.4f g\r\n", g_acc_calib_params.bias_z);
    printf("\r\n缩放因子 (Scale):\r\n");
    printf("  X: %.4f\r\n", g_acc_calib_params.scale_x);
    printf("  Y: %.4f\r\n", g_acc_calib_params.scale_y);
    printf("  Z: %.4f\r\n", g_acc_calib_params.scale_z);
    printf("\r\n提示: 参数已保存到 g_acc_calib_params，可保存到Flash\r\n");
    printf("=======================================\r\n\r\n");

    // 设置校准完成标志
    g_acc_calib_params.calibrated = 1;

    return 1;
}

/**
 * @brief       应用加速度计校准参数
 */
void imu_apply_acc_calibration(float ax_raw, float ay_raw, float az_raw,
                                float *ax_cal, float *ay_cal, float *az_cal)
{
    if (g_acc_calib_params.calibrated)
    {
        *ax_cal = (ax_raw - g_acc_calib_params.bias_x) * g_acc_calib_params.scale_x;
        *ay_cal = (ay_raw - g_acc_calib_params.bias_y) * g_acc_calib_params.scale_y;
        *az_cal = (az_raw - g_acc_calib_params.bias_z) * g_acc_calib_params.scale_z;
    }
    else
    {
        *ax_cal = ax_raw;
        *ay_cal = ay_raw;
        *az_cal = az_raw;
    }
}

/**
 * @brief       打印当前校准参数
 */
void imu_print_acc_calibration_params(void)
{
    printf("\r\n========== 加速度计校准参数 ==========\r\n");
    printf("校准状态: %s\r\n", g_acc_calib_params.calibrated ? "已校准" : "未校准");
    printf("\r\n零偏 (Bias):\r\n");
    printf("  bias_x  = %+.6f;  // g\r\n", g_acc_calib_params.bias_x);
    printf("  bias_y  = %+.6f;  // g\r\n", g_acc_calib_params.bias_y);
    printf("  bias_z  = %+.6f;  // g\r\n", g_acc_calib_params.bias_z);
    printf("\r\n缩放因子 (Scale):\r\n");
    printf("  scale_x = %.6f;\r\n", g_acc_calib_params.scale_x);
    printf("  scale_y = %.6f;\r\n", g_acc_calib_params.scale_y);
    printf("  scale_z = %.6f;\r\n", g_acc_calib_params.scale_z);
    printf("=====================================\r\n\r\n");
}

/**
 * @brief       重置校准参数
 */
void imu_reset_acc_calibration(void)
{
    g_acc_calib_params.bias_x = 0.0f;
    g_acc_calib_params.bias_y = 0.0f;
    g_acc_calib_params.bias_z = 0.0f;
    g_acc_calib_params.scale_x = 1.0f;
    g_acc_calib_params.scale_y = 1.0f;
    g_acc_calib_params.scale_z = 1.0f;
    g_acc_calib_params.calibrated = 0;

    printf("[信息] 加速度计校准参数已重置\r\n");
}

/**
 * @brief       验证校准效果
 */
float imu_verify_acc_calibration(uint32 test_duration_ms)
{
    if (!g_acc_calib_params.calibrated)
    {
        printf("[警告] 加速度计未校准，验证结果可能不准确\r\n");
    }

    printf("\r\n========== 加速度计校准验证 ==========\r\n");
    printf("测试时长: %lu ms\r\n", (unsigned long)test_duration_ms);
    printf("请保持设备静止...\r\n\r\n");

    uint32 start_time = system_getval_ms();
    uint16 sample_count = 0;
    float sum_error = 0.0f;
    float max_error = 0.0f;
    float min_error = 999.0f;

    while ((system_getval_ms() - start_time) < test_duration_ms)
    {
        // 等待数据更新
        imu_data.data_ready = false;
        while (!imu_data.data_ready && (system_getval_ms() - start_time) < test_duration_ms)
        {
            system_delay_ms(1);
        }

        if (!imu_data.data_ready)
            break;

        // 读取加速度并应用校准
        float ax = imu660rb_acc_transition(imu_data.acc_x);
        float ay = imu660rb_acc_transition(imu_data.acc_y);
        float az = imu660rb_acc_transition(imu_data.acc_z);

        float ax_cal, ay_cal, az_cal;
        imu_apply_acc_calibration(ax, ay, az, &ax_cal, &ay_cal, &az_cal);

        // 计算模长误差
        float norm = sqrtf(ax_cal * ax_cal + ay_cal * ay_cal + az_cal * az_cal);
        float error = fabsf(norm - 1.0f);

        sum_error += error;
        if (error > max_error) max_error = error;
        if (error < min_error) min_error = error;

        sample_count++;

        // 每500ms打印一次
        static uint32 last_print = 0;
        if ((system_getval_ms() - last_print) > 500)
        {
            printf("模长: %.4f g, 误差: %.4f g\r\n", norm, error);
            last_print = system_getval_ms();
        }
    }

    float avg_error = sum_error / sample_count;

    printf("\r\n========== 验证结果 ==========\r\n");
    printf("采样数: %d\r\n", sample_count);
    printf("平均误差: %.4f g\r\n", avg_error);
    printf("最大误差: %.4f g\r\n", max_error);
    printf("最小误差: %.4f g\r\n", min_error);
    printf("\r\n评估:\r\n");

    if (avg_error < 0.01f)
        printf("  [优秀] 校准精度很高!\r\n");
    else if (avg_error < 0.03f)
        printf("  [良好] 校准精度符合要求\r\n");
    else if (avg_error < 0.05f)
        printf("  [一般] 建议重新校准\r\n");
    else
        printf("  [较差] 需要重新校准\r\n");

    printf("===============================\r\n\r\n");

    return avg_error;
}

// *************************** 手动按钮确认模式 ***************************

/**
 * @brief       启动手动校准模式
 */
uint8 imu_calibrate_acc_manual(void)
{
    if (!imu_data.is_initialized)
    {
        printf("[ACC CAL] 错误: IMU未初始化!\r\n");
        return 0;
    }

    // 初始化状态
    memset(&g_manual_calib_state, 0, sizeof(g_manual_calib_state));
    g_manual_calib_state.is_active = true;
    g_manual_calib_state.target_samples = 30;  // 建议采集30个样本（覆盖6个方向，每方向5个样本）

    printf("\r\n========== 手动校准模式启动 ==========\r\n");
    printf("使用方法：\r\n");
    printf("  1. 将IMU旋转到一个方向并保持静止\r\n");
    printf("  2. 按下确认按钮采集当前方向数据\r\n");
    printf("  3. 重复步骤1-2，覆盖6个主要方向（±X, ±Y, ±Z）\r\n");
    printf("  4. 建议每个方向采集3-5次\r\n");
    printf("  5. 采集%d个样本后按任意键完成校准\r\n\r\n", g_manual_calib_state.target_samples);
    printf("目标样本数: %d\r\n", g_manual_calib_state.target_samples);
    printf("已采集: 0 | 方向覆盖: 0/6\r\n");
    printf("\r\n等待按钮确认...\r\n");
    printf("=====================================\r\n\r\n");

    return 1;
}

/**
 * @brief       按钮确认采样（在按键中断中调用）
 */
void imu_calibrate_acc_confirm_sample(void)
{
    if (!g_manual_calib_state.is_active)
    {
        printf("[警告] 校准模式未启动，请先调用 imu_calibrate_acc_manual()\r\n");
        return;
    }

    printf("\r\n[采样] 按钮确认，开始采集...\r\n");

    // ========== 快速连续采集10个样本取平均 ==========
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    uint8 valid_samples = 0;

    for (uint8 i = 0; i < ACC_CAL_STILLNESS_SAMPLES; i++)
    {
        // 等待数据更新
        imu_data.data_ready = false;
        uint32 wait_start = system_getval_ms();
        while (!imu_data.data_ready)
        {
            if ((system_getval_ms() - wait_start) > 10)
                break;
        }

        if (!imu_data.data_ready)
            continue;

        float ax = imu660rb_acc_transition(imu_data.acc_x);
        float ay = imu660rb_acc_transition(imu_data.acc_y);
        float az = imu660rb_acc_transition(imu_data.acc_z);

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        valid_samples++;
    }

    if (valid_samples < 5)
    {
        printf("[错误] 采样失败，有效样本数不足（%d/10）\r\n", valid_samples);
        return;
    }

    // 计算平均值
    float ax_avg = ax_sum / valid_samples;
    float ay_avg = ay_sum / valid_samples;
    float az_avg = az_sum / valid_samples;

    printf("[采样] 完成，样本数: %d, 加速度: [%.3f, %.3f, %.3f] g\r\n",
           valid_samples, ax_avg, ay_avg, az_avg);

    // ========== 更新方向覆盖统计 ==========
    update_direction_coverage(ax_avg, ay_avg, az_avg, &g_manual_calib_state.coverage);

    // ========== 增量式累加 H'H 和 H'b ==========
    float h[6] = {ax_avg * ax_avg, ay_avg * ay_avg, az_avg * az_avg,
                  ax_avg, ay_avg, az_avg};
    float b_val = 1.0f;

    for (uint8 i = 0; i < 6; i++)
    {
        for (uint8 j = 0; j < 6; j++)
        {
            g_manual_calib_state.HTH_data[i * 6 + j] += h[i] * h[j];
        }
    }

    for (uint8 i = 0; i < 6; i++)
    {
        g_manual_calib_state.HTb_data[i] += h[i] * b_val;
    }

    g_manual_calib_state.sample_count++;

    // 打印进度
    printf("\r\n--- 当前进度 ---\r\n");
    printf("已采集: %d/%d\r\n", g_manual_calib_state.sample_count, g_manual_calib_state.target_samples);
    printf("方向覆盖: ");
    print_direction_coverage(&g_manual_calib_state.coverage);
    printf(" (%d/6)\r\n", g_manual_calib_state.coverage.count);

    if (g_manual_calib_state.coverage.count < 6)
    {
        printf("缺失方向: ");
        if (!g_manual_calib_state.coverage.covered_px) printf("+X ");
        if (!g_manual_calib_state.coverage.covered_nx) printf("-X ");
        if (!g_manual_calib_state.coverage.covered_py) printf("+Y ");
        if (!g_manual_calib_state.coverage.covered_ny) printf("-Y ");
        if (!g_manual_calib_state.coverage.covered_pz) printf("+Z ");
        if (!g_manual_calib_state.coverage.covered_nz) printf("-Z ");
        printf("\r\n");
    }

    printf("---------------\r\n\r\n");

    // 判断是否可以完成校准
    if (g_manual_calib_state.sample_count >= g_manual_calib_state.target_samples)
    {
        printf("[提示] 已达到目标样本数！可以调用 imu_calibrate_acc_manual_finish() 完成校准\r\n\r\n");
    }
}

/**
 * @brief       完成手动校准并计算参数
 */
uint8 imu_calibrate_acc_manual_finish(void)
{
    if (!g_manual_calib_state.is_active)
    {
        printf("[错误] 校准模式未启动\r\n");
        return 0;
    }

    if (g_manual_calib_state.sample_count < 20)
    {
        printf("[错误] 样本数不足（%d/20），至少需要20个样本\r\n", g_manual_calib_state.sample_count);
        return 0;
    }

    if (g_manual_calib_state.coverage.count < 4)
    {
        printf("[警告] 方向覆盖不足（%d/6），建议至少覆盖4个方向\r\n", g_manual_calib_state.coverage.count);
        printf("是否继续？这可能影响校准精度。\r\n");
        // 这里可以等待用户确认，暂时继续执行
    }

    printf("\r\n========== 开始计算校准参数 ==========\r\n");
    printf("总样本数: %d\r\n", g_manual_calib_state.sample_count);
    printf("方向覆盖: %d/6\r\n\r\n", g_manual_calib_state.coverage.count);

    // ========== 求解最小二乘 X = (H'H)^(-1) * H'b ==========
    float HTH_inv_data[36];
    float X_data[6];

    arm_matrix_instance_f32 HTH, HTH_inv, HTb, X;
    arm_mat_init_f32(&HTH, 6, 6, g_manual_calib_state.HTH_data);
    arm_mat_init_f32(&HTH_inv, 6, 6, HTH_inv_data);
    arm_mat_init_f32(&HTb, 6, 1, g_manual_calib_state.HTb_data);
    arm_mat_init_f32(&X, 6, 1, X_data);

    // 计算 (H'H)^(-1)
    if (arm_mat_inverse_f32(&HTH, &HTH_inv) != ARM_MATH_SUCCESS)
    {
        printf("[错误] 矩阵求逆失败! (数据可能不够分散)\r\n");
        printf("建议: 在更多方向上采集数据\r\n");
        g_manual_calib_state.is_active = false;
        return 0;
    }

    // 计算 X = (H'H)^(-1) * H'b
    if (arm_mat_mult_f32(&HTH_inv, &HTb, &X) != ARM_MATH_SUCCESS)
    {
        printf("[错误] 最终求解失败!\r\n");
        g_manual_calib_state.is_active = false;
        return 0;
    }

    // ========== 提取椭球参数并转换为校准参数 ==========
    float A = X_data[0];
    float B = X_data[1];
    float C = X_data[2];
    float D = X_data[3];
    float E = X_data[4];
    float F = X_data[5];

    if (A <= 1e-6f || B <= 1e-6f || C <= 1e-6f)
    {
        printf("[错误] 拟合结果异常 (系数过小)!\r\n");
        g_manual_calib_state.is_active = false;
        return 0;
    }

    // 计算校准参数
    g_acc_calib_params.bias_x = -D / (2.0f * A);
    g_acc_calib_params.bias_y = -E / (2.0f * B);
    g_acc_calib_params.bias_z = -F / (2.0f * C);
    g_acc_calib_params.scale_x = 1.0f / sqrtf(A);
    g_acc_calib_params.scale_y = 1.0f / sqrtf(B);
    g_acc_calib_params.scale_z = 1.0f / sqrtf(C);

    // 参数合理性检查
    if (fabsf(g_acc_calib_params.bias_x) > 2.0f ||
        fabsf(g_acc_calib_params.bias_y) > 2.0f ||
        fabsf(g_acc_calib_params.bias_z) > 2.0f ||
        g_acc_calib_params.scale_x < 0.5f || g_acc_calib_params.scale_x > 2.0f ||
        g_acc_calib_params.scale_y < 0.5f || g_acc_calib_params.scale_y > 2.0f ||
        g_acc_calib_params.scale_z < 0.5f || g_acc_calib_params.scale_z > 2.0f)
    {
        printf("[错误] 参数超出合理范围!\r\n");
        printf("Bias: [%.3f, %.3f, %.3f] (期望: ±2.0g)\r\n",
               g_acc_calib_params.bias_x, g_acc_calib_params.bias_y, g_acc_calib_params.bias_z);
        printf("Scale: [%.3f, %.3f, %.3f] (期望: 0.5~2.0)\r\n",
               g_acc_calib_params.scale_x, g_acc_calib_params.scale_y, g_acc_calib_params.scale_z);
        g_manual_calib_state.is_active = false;
        return 0;
    }

    // ========== 输出校准结果 ==========
    printf("\r\n========== 校准成功! ==========\r\n");
    printf("零偏 (Bias):\r\n");
    printf("  X: %+.4f g\r\n", g_acc_calib_params.bias_x);
    printf("  Y: %+.4f g\r\n", g_acc_calib_params.bias_y);
    printf("  Z: %+.4f g\r\n", g_acc_calib_params.bias_z);
    printf("\r\n缩放因子 (Scale):\r\n");
    printf("  X: %.4f\r\n", g_acc_calib_params.scale_x);
    printf("  Y: %.4f\r\n", g_acc_calib_params.scale_y);
    printf("  Z: %.4f\r\n", g_acc_calib_params.scale_z);
    printf("\r\n提示: 参数已保存，可保存到Flash\r\n");
    printf("====================================\r\n\r\n");

    g_acc_calib_params.calibrated = 1;
    g_manual_calib_state.is_active = false;

    return 1;
}

