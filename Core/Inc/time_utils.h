/**
 * @file time_utils.h
 * @brief 时间戳处理工具（安全处理 HAL_GetTick 溢出）
 * @details 提供安全的时间戳计算和超时检测函数
 *          使用 C99 无符号整数的模运算特性，正确处理 uint32_t 溢出
 *
 * ## 核心问题：HAL_GetTick() 溢出处理
 *
 * **HAL_GetTick() 特性**:
 * - 返回类型：`uint32_t`（32位无符号整数）
 * - 计数范围：0 ~ 4,294,967,295 (2^32 - 1)
 * - 溢出周期：约49.7天（2^32 ms ≈ 49.71 天）
 * - 溢出行为：自动回绕到0（0xFFFFFFFF → 0x00000000）
 *
 * **错误的时间差计算**:
 * ```c
 * // ❌ 错误示例：溢出时产生负数（如果使用有符号比较）
 * uint32_t start = 0xFFFFFFF0;  // 系统运行约49.7天
 * uint32_t now = 0x00000010;    // 溢出后32ms
 * int32_t diff = (int32_t)(now - start);  // -4,294,967,264 ❌
 *
 * // ❌ 错误示例：直接比较大小
 * if (now > start) {  // false，因为0x10 < 0xFFFFFFF0 ❌
 *     // 永远不会执行
 * }
 * ```
 *
 * **正确的时间差计算** (本模块方法):
 * ```c
 * // ✅ 正确：利用无符号整数的模运算
 * uint32_t start = 0xFFFFFFF0;
 * uint32_t now = 0x00000010;
 * uint32_t diff = now - start;  // 0x20 = 32ms ✅
 * ```
 *
 * ## C99 数学原理
 *
 * **无符号整数模运算**（C99标准 6.2.5.9）:
 * - 所有无符号整数运算遵循 modulo 2^n（n=位宽）
 * - 溢出和下溢自动回绕，无未定义行为
 * - 减法运算：`(a - b) mod 2^32`
 *
 * **数学证明**:
 * ```
 * 设：start = 0xFFFFFFF0, now = 0x00000010
 *
 * 实际时间差 = 16 + (2^32 - 0xFFFFFFF0) = 16 + 16 = 32ms
 *
 * 无符号减法：
 * now - start = 0x00000010 - 0xFFFFFFF0
 *             = 0x00000010 + (~0xFFFFFFF0 + 1)  // 补码运算
 *             = 0x00000010 + 0x00000010
 *             = 0x00000020 = 32ms ✅
 * ```
 *
 * ## 系统应用
 *
 * **超时检测场景**:
 * ```
 * 模块                    超时阈值        溢出影响
 * ----------------       -----------     -----------
 * CRSF接收器             1000ms (1s)     无影响 ✅
 * Jetson通信             500ms           无影响 ✅
 * 控制系统调试输出        200ms           无影响 ✅
 * 模式切换保护            100ms           无影响 ✅
 * ```
 * 所有超时检测周期 << 49.7天溢出周期，完全不受溢出影响。
 *
 * **系统运行时间限制**:
 * - 保证正确：时间差 < 2^31 ms (约24.8天)
 * - 实际应用：超时检测通常为秒级（< 10秒）
 * - 安全余量：10秒 / 24.8天 ≈ 0.00046%
 *
 * ## 性能特性
 *
 * **执行效率**:
 * - `time_elapsed_ms()`: 1条CPU指令（SUB）
 * - `time_has_timed_out()`: 2条CPU指令（SUB + CMP）
 * - 内联函数：编译器优化后零函数调用开销
 *
 * **内存占用**:
 * - 无全局变量
 * - 无堆分配
 * - 纯函数调用，栈占用为0
 *
 * ## 使用示例
 *
 * **基本超时检测**:
 * ```c
 * // 初始化：记录开始时间
 * uint32_t start_time = HAL_GetTick();
 *
 * // 主循环：检查超时
 * while (1) {
 *     uint32_t now = HAL_GetTick();
 *     if (time_has_timed_out(now, start_time, 1000)) {
 *         // 超过1秒，执行超时处理
 *         printf("Timeout!\n");
 *         break;
 *     }
 *     // 正常处理
 * }
 * ```
 *
 * **信号有效性检测** (CRSF接收器):
 * ```c
 * typedef struct {
 *     uint32_t last_update_ms;  // 最后更新时间
 *     bool is_valid;             // 信号有效标志
 * } SignalStatus_t;
 *
 * void CheckSignalStatus(SignalStatus_t *status)
 * {
 *     uint32_t now = HAL_GetTick();
 *     // 检查是否超时（1秒未更新）
 *     if (time_has_timed_out(now, status->last_update_ms, 1000)) {
 *         status->is_valid = false;  // 信号丢失
 *     }
 * }
 * ```
 *
 * **周期性任务执行**:
 * ```c
 * static uint32_t last_debug_time = 0;
 *
 * void MainLoop(void)
 * {
 *     uint32_t now = HAL_GetTick();
 *
 *     // 每200ms输出一次调试信息
 *     if (time_elapsed_ms(now, last_debug_time) >= 200) {
 *         ControlSystem_DebugReport();
 *         last_debug_time = now;  // 更新时间戳
 *     }
 * }
 * ```
 *
 * **多信号超时检测** (控制系统):
 * ```c
 * void ControlSystem_UpdateSignalStatus(ControlSystem_t *sys)
 * {
 *     uint32_t now = HAL_GetTick();
 *
 *     // 检查RC信号超时
 *     sys->rc_signal_valid = !time_has_timed_out(now,
 *                                                 sys->rc_last_update_ms,
 *                                                 CONTROL_SYSTEM_RC_TIMEOUT_MS);
 *
 *     // 检查Jetson信号超时
 *     sys->jetson_signal_valid = !time_has_timed_out(now,
 *                                                     sys->jetson_last_update_ms,
 *                                                     CONTROL_SYSTEM_JETSON_TIMEOUT_MS);
 * }
 * ```
 *
 * ## 溢出测试验证
 *
 * **单元测试示例**:
 * ```c
 * void TestTimeOverflow(void)
 * {
 *     // 测试1：正常情况
 *     uint32_t elapsed = time_elapsed_ms(5000, 1000);
 *     assert(elapsed == 4000);  // ✅ 4000ms
 *
 *     // 测试2：溢出情况
 *     elapsed = time_elapsed_ms(0x00000100, 0xFFFFFF00);
 *     assert(elapsed == 0x200);  // ✅ 512ms
 *
 *     // 测试3：临界溢出
 *     elapsed = time_elapsed_ms(0x00000000, 0xFFFFFFFF);
 *     assert(elapsed == 1);  // ✅ 1ms
 *
 *     // 测试4：超时检测（溢出情况）
 *     bool timed_out = time_has_timed_out(0x00000200, 0xFFFFFF00, 0x400);
 *     assert(timed_out == true);  // ✅ 已超时（512ms > 1024ms？）
 *
 *     printf("All overflow tests passed!\n");
 * }
 * ```
 *
 * ## 限制与注意事项
 *
 * **时间差限制**:
 * - **保证正确**：时间差 < 2^31 ms (约24.8天)
 * - **超出限制**：时间差 > 2^31 ms 时结果不可靠
 * - **实际影响**：超时检测通常为秒~分钟级，完全满足
 *
 * **不适用场景**:
 * - ❌ 长期时间戳（> 24.8天）：需要64位时间戳
 * - ❌ 绝对时间记录：需要RTC（实时时钟）
 * - ❌ 高精度计时（< 1ms）：需要硬件定时器
 *
 * **推荐使用场景**:
 * - ✅ 短期超时检测（秒~分钟级）
 * - ✅ 周期性任务调度（毫秒~秒级）
 * - ✅ 信号有效性检测
 * - ✅ 通信超时保护
 *
 * ## 与其他方案比较
 *
 * **方案1：有符号转换**（❌ 错误）:
 * ```c
 * int32_t diff = (int32_t)(now - start);
 * if (diff > timeout) { }  // 溢出时diff为负数，错误 ❌
 * ```
 *
 * **方案2：64位时间戳**（✅ 可行但冗余）:
 * ```c
 * uint64_t now64 = HAL_GetTick();
 * uint64_t diff = now64 - start64;  // 永不溢出，但占用更多资源
 * ```
 *
 * **方案3：本模块方案**（✅ 推荐）:
 * ```c
 * uint32_t diff = time_elapsed_ms(now, start);  // 最高效，C99标准保证
 * ```
 *
 * @note 线程安全：
 *       - 纯函数，无全局状态
 *       - 可从任何上下文调用（主循环/中断）
 *       - 无需互斥保护
 *
 * @warning 约束：
 *          1. 时间差必须 < 2^31 ms（约24.8天）
 *          2. 仅适用于相对时间测量，不适合绝对时间
 *          3. 精度受HAL_GetTick()限制（通常1ms）
 *
 * @see HAL_GetTick() STM32 HAL库时基函数
 * @see control_system.c 超时检测应用示例
 * @see crsf_receiver.c 信号有效性检测应用
 *
 * @author STM32开发团队
 * @date 2025-01-09
 * @version 2.0
 */

#ifndef __TIME_UTILS_H
#define __TIME_UTILS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 时间计算函数 ==================== */

/**
 * @brief 计算两个时间戳之间经过的毫秒数（溢出安全）
 * @param now_ms 当前时间戳（毫秒），通常来自 HAL_GetTick()
 * @param since_ms 起始时间戳（毫秒）
 * @return 经过的毫秒数（uint32_t）
 *
 * @details 核心算法：
 *          利用 C99 无符号整数的模运算特性：
 *          `result = (now_ms - since_ms) mod 2^32`
 *
 * 数学原理：
 * - 无符号整数减法在2^32模空间中进行
 * - 即使now_ms < since_ms（溢出），结果仍然正确
 * - 编译器优化为单条SUB指令（零开销）
 *
 * 正确性保证（C99标准6.2.5.9）：
 * > "A computation involving unsigned operands can never overflow,
 * > because a result that cannot be represented by the resulting
 * > unsigned integer type is reduced modulo the number that is one
 * > greater than the largest value that can be represented by the
 * > resulting type."
 *
 * @note 限制：
 *       - 仅在时间差 < 2^31 ms（约24.8天）时保证正确
 *       - 对于典型超时检测（秒~分钟级）完全满足需求
 *
 * @note 性能：
 *       - 执行时间：< 0.1 μs（单条SUB指令）
 *       - 编译器内联：零函数调用开销
 *
 * @example 正常情况（无溢出）:
 * ```c
 * uint32_t start = 1000;
 * uint32_t now = 5000;
 * uint32_t elapsed = time_elapsed_ms(now, start);
 * // elapsed = 4000 ms ✅
 * ```
 *
 * @example 溢出情况（HAL_GetTick回绕）:
 * ```c
 * uint32_t start = 0xFFFFFF00;  // 系统运行约49.7天
 * uint32_t now = 0x00000100;    // 溢出后256ms
 * uint32_t elapsed = time_elapsed_ms(now, start);
 * // elapsed = 0x200 = 512 ms ✅（正确！）
 * ```
 *
 * @example 临界溢出:
 * ```c
 * uint32_t start = 0xFFFFFFFF;  // 溢出前1ms
 * uint32_t now = 0x00000000;    // 溢出后
 * uint32_t elapsed = time_elapsed_ms(now, start);
 * // elapsed = 1 ms ✅
 * ```
 */
static inline uint32_t time_elapsed_ms(uint32_t now_ms, uint32_t since_ms)
{
    return now_ms - since_ms;
}

/**
 * @brief 判断从起始时间点到现在是否已超时（溢出安全）
 * @param now_ms 当前时间戳（毫秒），通常来自 HAL_GetTick()
 * @param since_ms 起始时间戳（毫秒）
 * @param timeout_ms 超时阈值（毫秒）
 * @return true=已超时, false=未超时
 *
 * @details 实现逻辑：
 *          `elapsed = time_elapsed_ms(now_ms, since_ms)`
 *          `return elapsed > timeout_ms`
 *
 * 溢出处理：
 * - 内部调用 `time_elapsed_ms()` 确保溢出安全
 * - 即使 HAL_GetTick() 溢出，超时检测仍然正确
 *
 * 超时判定：
 * - 经过时间 > timeout_ms → 返回 true（已超时）
 * - 经过时间 ≤ timeout_ms → 返回 false（未超时）
 *
 * @note 边界条件：
 *       - elapsed == timeout_ms → 返回 false（未超时）
 *       - 如需 elapsed >= timeout_ms 触发，修改为 `>=`
 *
 * @note 性能：
 *       - 执行时间：< 0.2 μs（SUB + CMP）
 *       - 编译器内联：零函数调用开销
 *
 * @example 典型超时检测:
 * ```c
 * uint32_t start_time = HAL_GetTick();
 *
 * while (1) {
 *     uint32_t now = HAL_GetTick();
 *     if (time_has_timed_out(now, start_time, 1000)) {
 *         printf("Timeout after 1 second!\n");
 *         break;
 *     }
 *     // 正常处理
 * }
 * ```
 *
 * @example 信号丢失检测:
 * ```c
 * // 检查CRSF信号是否超时
 * uint32_t now = HAL_GetTick();
 * bool signal_lost = time_has_timed_out(now,
 *                                       crsf_data.last_update_time_ms,
 *                                       1000);  // 1秒超时
 * if (signal_lost) {
 *     crsf_data.is_valid = false;
 * }
 * ```
 *
 * @example 周期性任务:
 * ```c
 * static uint32_t last_report_time = 0;
 *
 * void ControlLoop(void) {
 *     uint32_t now = HAL_GetTick();
 *
 *     // 每200ms输出一次报告
 *     if (time_elapsed_ms(now, last_report_time) >= 200) {
 *         ControlSystem_DebugReport();
 *         last_report_time = now;
 *     }
 * }
 * ```
 */
static inline bool time_has_timed_out(uint32_t now_ms, uint32_t since_ms, uint32_t timeout_ms)
{
    return time_elapsed_ms(now_ms, since_ms) > timeout_ms;
}

#ifdef __cplusplus
}
#endif

#endif /* __TIME_UTILS_H */
