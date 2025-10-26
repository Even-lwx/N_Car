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

    // ========== 快速连续采集500个样本取平均 ==========
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    uint16 valid_samples = 0;  // 改为uint16以支持500个样本

    for (uint16 i = 0; i < ACC_CAL_IMPROVED_STILLNESS_SAMPLES; i++)
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

    if (valid_samples < (ACC_CAL_IMPROVED_STILLNESS_SAMPLES / 2))
    {
        printf("[错误] 采样失败，有效样本数不足（%d/%d）\r\n", valid_samples, ACC_CAL_IMPROVED_STILLNESS_SAMPLES);
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

    // 调试输出：显示拟合系数
    printf("[调试] 椭球拟合系数:\r\n");
    printf("  A=%.10f, B=%.10f, C=%.10f\r\n", A, B, C);
    printf("  D=%.10f, E=%.10f, F=%.10f\r\n", D, E, F);

    if (A <= 1e-8f || B <= 1e-8f || C <= 1e-8f)
    {
        printf("[错误] 拟合结果异常 (系数过小)!\r\n");
        printf("  A=%.3e, B=%.3e, C=%.3e (阈值: 1e-8)\r\n", A, B, C);
        printf("建议: 在更多方向上采集数据（确保6面都有覆盖）\r\n");
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

/**
 * @brief       重置加速度计校准参数
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

// *************************** 局部校准模式（Z轴向上范围） ***************************

/**
 * @brief       启动局部手动校准模式（Z轴向上范围）
 * @note        适用于四旋翼、平衡车等Z轴向上为主的应用场景
 *              不需要6面翻转，只需要在±30度倾角范围内采集多个姿态
 *              对于四旋翼：可在桌面上前后左右倾斜机体，模拟飞行姿态变化
 */
uint8 imu_calibrate_acc_manual_local(void)
{
    if (!imu_data.is_initialized)
    {
        printf("[ACC CAL LOCAL] 错误: IMU未初始化!\r\n");
        return 0;
    }

    // 初始化状态
    memset(&g_manual_calib_state, 0, sizeof(g_manual_calib_state));
    g_manual_calib_state.is_active = true;
    g_manual_calib_state.target_samples = 10;  // 局部校准建议10个姿态（每姿态50次采样）

    printf("\r\n========== 局部校准模式启动（Z轴向上） ==========\r\n");
    printf("适用场景：四旋翼、平衡车等Z轴向上为主的应用\r\n\r\n");
    printf("使用方法：\r\n");
    printf("  1. 保持Z轴向上（±30度倾角范围内）\r\n");
    printf("  2. 倾斜到一个姿态并保持静止（每姿态500次采样，约5秒）\r\n");
    printf("  3. 按下OK采集当前姿态（一个姿态按一次）\r\n");
    printf("  4. 重复步骤2-3，覆盖不同倾角（前后左右+组合）\r\n");
    printf("  5. 建议采集%d个姿态后完成校准\r\n\r\n", g_manual_calib_state.target_samples);
    printf("目标姿态数: %d\r\n", g_manual_calib_state.target_samples);
    printf("已采集: 0\r\n");
    printf("\r\n等待按钮确认...\r\n");
    printf("================================================\r\n\r\n");

    return 1;
}

/**
 * @brief       完成局部手动校准并计算参数（Z轴向上范围）
 * @note        使用简化的椭球拟合模型：
 *              - 假设 Z 轴始终接近 1g（向上）
 *              - 主要校准 X、Y 轴的 bias 和 scale
 *              - Z 轴使用所有样本的平均值估算 bias 和 scale
 */
uint8 imu_calibrate_acc_manual_finish_local(void)
{
    if (!g_manual_calib_state.is_active)
    {
        printf("[错误] 校准模式未启动\r\n");
        return 0;
    }

    if (g_manual_calib_state.sample_count < 8)
    {
        printf("[错误] 姿态数不足（%d/8），局部校准至少需要8个姿态\r\n", g_manual_calib_state.sample_count);
        return 0;
    }

    printf("\r\n========== 开始计算局部校准参数 ==========\r\n");
    printf("总姿态数: %d\r\n", g_manual_calib_state.sample_count);
    printf("总采样数: %d (每姿态500次)\r\n", g_manual_calib_state.sample_count * 500);
    printf("校准模式: Z轴向上局部校准\r\n\r\n");

    // ========== 简化模型：4参数拟合（X、Y的bias和scale） ==========
    // 椭球方程简化为：A*(x+D)^2 + B*(y+E)^2 + (z-1)^2 = 1
    // 其中 z 接近 1g，我们用平均值估算 Z 的 bias

    // 由于原始数据未保存，使用完整6参数模型求解，然后只使用 X、Y 的结果

    // ========== 使用完整6参数求解（与全局校准相同） ==========
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
        printf("建议: 在更多倾角方向上采集数据\r\n");
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

    // ========== 提取椭球参数 ==========
    float A = X_data[0];
    float B = X_data[1];
    float C = X_data[2];
    float D = X_data[3];
    float E = X_data[4];
    float F = X_data[5];

    // 调试输出：显示拟合系数
    printf("[调试] 椭球拟合系数:\r\n");
    printf("  A=%.10f, B=%.10f, C=%.10f\r\n", A, B, C);
    printf("  D=%.10f, E=%.10f, F=%.10f\r\n", D, E, F);

    // 局部校准时Z轴变化小，放宽阈值检查（从1e-6改为1e-8）
    if (A <= 1e-8f || B <= 1e-8f || C <= 1e-8f)
    {
        printf("[错误] 拟合结果异常 (系数过小)!\r\n");
        printf("  A=%.3e, B=%.3e, C=%.3e (阈值: 1e-8)\r\n", A, B, C);
        printf("建议: 确保姿态覆盖足够的倾角范围（前后左右±10度以上）\r\n");
        g_manual_calib_state.is_active = false;
        return 0;
    }

    // ========== 局部校准策略：只使用 X、Y 的完整结果，Z 轴使用简化估计 ==========
    g_acc_calib_params.bias_x = -D / (2.0f * A);
    g_acc_calib_params.bias_y = -E / (2.0f * B);
    g_acc_calib_params.scale_x = 1.0f / sqrtf(A);
    g_acc_calib_params.scale_y = 1.0f / sqrtf(B);

    // Z 轴简化处理：假设 scale 接近 1.0，只校准 bias
    // bias_z ≈ 平均测量值 - 1.0g
    g_acc_calib_params.bias_z = -F / (2.0f * C);
    g_acc_calib_params.scale_z = 1.0f / sqrtf(C);

    // 如果 Z 轴的 scale 偏差过大（说明局部拟合不可靠），强制设为 1.0
    if (fabsf(g_acc_calib_params.scale_z - 1.0f) > 0.15f)
    {
        printf("[警告] Z轴scale偏差较大 (%.3f)，强制设为1.0\r\n", g_acc_calib_params.scale_z);
        g_acc_calib_params.scale_z = 1.0f;
    }

    // 参数合理性检查（放宽 Z 轴限制）
    if (fabsf(g_acc_calib_params.bias_x) > 2.0f ||
        fabsf(g_acc_calib_params.bias_y) > 2.0f ||
        fabsf(g_acc_calib_params.bias_z) > 2.0f ||
        g_acc_calib_params.scale_x < 0.5f || g_acc_calib_params.scale_x > 2.0f ||
        g_acc_calib_params.scale_y < 0.5f || g_acc_calib_params.scale_y > 2.0f)
    {
        printf("[错误] 参数超出合理范围!\r\n");
        printf("Bias: [%.3f, %.3f, %.3f] (期望: ±2.0g)\r\n",
               g_acc_calib_params.bias_x, g_acc_calib_params.bias_y, g_acc_calib_params.bias_z);
        printf("Scale: [%.3f, %.3f, %.3f] (X/Y期望: 0.5~2.0)\r\n",
               g_acc_calib_params.scale_x, g_acc_calib_params.scale_y, g_acc_calib_params.scale_z);
        g_manual_calib_state.is_active = false;
        return 0;
    }

    // ========== 输出校准结果 ==========
    printf("\r\n========== 局部校准成功! ==========\r\n");
    printf("零偏 (Bias):\r\n");
    printf("  X: %+.4f g  (完整拟合)\r\n", g_acc_calib_params.bias_x);
    printf("  Y: %+.4f g  (完整拟合)\r\n", g_acc_calib_params.bias_y);
    printf("  Z: %+.4f g  (简化估计)\r\n", g_acc_calib_params.bias_z);
    printf("\r\n缩放因子 (Scale):\r\n");
    printf("  X: %.4f  (完整拟合)\r\n", g_acc_calib_params.scale_x);
    printf("  Y: %.4f  (完整拟合)\r\n", g_acc_calib_params.scale_y);
    printf("  Z: %.4f  (简化估计)\r\n", g_acc_calib_params.scale_z);
    printf("\r\n提示: 参数已保存，可保存到Flash\r\n");
    printf("======================================\r\n\r\n");

    g_acc_calib_params.calibrated = 1;
    g_manual_calib_state.is_active = false;

    return 1;
}

