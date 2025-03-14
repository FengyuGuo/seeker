#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/types.h>

#define PPS_GPIO 492  // 假设 PPS 信号通过 GPIO 18 输出
#define PULSE_WIDTH_NS 100000000  // 拉低延迟时间100ms（按需调整）

typedef struct {
    struct completion data_ready;
    struct timespec64 pps_timestamp;
    struct mutex lock;
    struct hrtimer pps_timer;
    struct work_struct pps_work;
} pps_ctx_t;

static ktime_t pps_interval;
static pps_ctx_t gst_pps_ctx;

// 工作队列处理函数（允许休眠）
static void pps_work_handler(struct work_struct *work) {
    usleep_range(PULSE_WIDTH_NS/1000, PULSE_WIDTH_NS/1000 + 100); // 微秒级休眠
    gpio_set_value(PPS_GPIO, 0); // 拉低GPIO
    complete(&gst_pps_ctx.data_ready);
}

static enum hrtimer_restart pps_timer_callback(struct hrtimer *timer)
{
    /* 切换 GPIO 状态 */
    gpio_set_value(PPS_GPIO, 1);

    /* 重新启动定时器 */
    hrtimer_forward_now(timer, pps_interval);
   // 记录时间戳
    mutex_lock(&gst_pps_ctx.lock);
    /* 获取当前时间戳 */
    ktime_get_real_ts64(&gst_pps_ctx.pps_timestamp);
    mutex_unlock(&gst_pps_ctx.lock);
    schedule_work(&gst_pps_ctx.pps_work);     // 调度下半部
    return HRTIMER_RESTART;
}

// Misc设备文件操作
static int pps_open(struct inode *inode, struct file *filp)
{
    /* 启动定时器 */
    hrtimer_start(&gst_pps_ctx.pps_timer, pps_interval, HRTIMER_MODE_REL);

    return 0;
}

static ssize_t pps_read(struct file *filp, char __user *buf,
                       size_t count, loff_t *pos) {
    int ret;
    struct timespec64 ts;

    // 等待时间戳就绪（可中断等待）
    ret = wait_for_completion_interruptible(&gst_pps_ctx.data_ready);
    if (ret)
        return ret;

    mutex_lock(&gst_pps_ctx.lock);
    ts = gst_pps_ctx.pps_timestamp;
    reinit_completion(&gst_pps_ctx.data_ready);
    mutex_unlock(&gst_pps_ctx.lock);

    /* 将时间戳复制到用户空间 */
    if (copy_to_user(buf, &ts, sizeof(struct timespec64))) {
        return -EFAULT;
    }

    return sizeof(struct timespec64);
}

static int pps_r_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations pps_fops = {
    .owner = THIS_MODULE,
    .open = pps_open,
    .read = pps_read,
    .release = pps_r_release,
};

// 定义Misc设备
static struct miscdevice pps_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "pps_s",
    .fops = &pps_fops,
};

static int __init pps_s_module_init(void)
{
    int ret;

    /* 申请 GPIO */
    ret = gpio_request(PPS_GPIO, "PAC.06");
    if (ret) {
        pr_err("Failed to request GPIO %d\n", PPS_GPIO);
        return ret;
    }

    /* 设置 GPIO 为输出 */
    ret = gpio_direction_output(PPS_GPIO, 0);
    if (ret) {
        pr_err("Failed to set GPIO %d as output\n", PPS_GPIO);
        goto __GPIO_REQ_FAIL__;
    }

    // 注册Misc设备
    ret = misc_register(&pps_miscdev);
    if (ret) {
        pr_err("Failed to register misc device\n");
        goto __MISC_REGISTER_FAIL__;
    }

    // 初始化工作队列
    INIT_WORK(&gst_pps_ctx.pps_work, pps_work_handler);
    mutex_init(&gst_pps_ctx.lock);
    init_completion(&gst_pps_ctx.data_ready);
    /* 初始化定时器 */
    hrtimer_init(&gst_pps_ctx.pps_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    gst_pps_ctx.pps_timer.function = pps_timer_callback;

    /* 设置定时器间隔为 1 秒 */
    pps_interval = ktime_set(1, 0);  // 1 秒


    pr_info("PPS signal generator initialized\n");
    return 0;

__MISC_REGISTER_FAIL__:
__GPIO_REQ_FAIL__:
    gpio_free(PPS_GPIO);

    return ret;
}

static void __exit pps_s_module_exit(void)
{
    /* 停止定时器 */
    hrtimer_cancel(&gst_pps_ctx.pps_timer);
    cancel_work_sync(&gst_pps_ctx.pps_work);
    /* 释放 GPIO */
    gpio_set_value(PPS_GPIO, 0);
    gpio_free(PPS_GPIO);
    misc_deregister(&pps_miscdev);

    pr_info("PPS signal generator exited\n");
}

module_init(pps_s_module_init);
module_exit(pps_s_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("seeker");
MODULE_DESCRIPTION("A simple PPS signal generator driver");
