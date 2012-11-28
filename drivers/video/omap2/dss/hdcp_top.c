/*
 * hdcp_top.c
 *
 * HDCP interface DSS driver setting for TI's OMAP4 family of processor.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Sujeet Baranwal <s-obaranwal@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <linux/firmware.h>
#include "ti_hdmi_4xxx_ip.h"
#include "../dss/dss.h"
#include "hdcp.h"

struct hdcp_worker_data {
	struct delayed_work dwork;
	atomic_t state;
};

struct hdcp_data {
	void __iomem *deshdcp_base_addr;
	struct mutex lock;
	struct hdcp_enable_control *en_ctrl;
	struct workqueue_struct *workqueue;
	struct hdcp_worker_data *hdcp_work;
	struct miscdevice *mdev;
	bool hdcp_keys_loaded;
	int hdcp_up_event;
	int hdcp_down_event;
	/*
	 * This lock is to protect hdcp wait ioctl being called by multiple
	 * process incorrectly. This can happen if the HDCP user space daemon
	 * is mistakenly called more than once.
	 */
	struct mutex re_entrant_lock;
	bool re_entrance_flag;
};

static struct hdcp_data *hdcp;

static void hdcp_work_queue(struct work_struct *work);

static DECLARE_WAIT_QUEUE_HEAD(hdcp_up_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(hdcp_down_wait_queue);

static int hdcp_step2_authenticate_repeater(int flags)
{
	int ret;

	HDCP_DBG("Wait for user space task %x\n", flags);
	hdcp->hdcp_up_event = flags & 0xFF;
	hdcp->hdcp_down_event = flags & 0xFF;
	wake_up_interruptible(&hdcp_up_wait_queue);
	wait_event_interruptible(hdcp_down_wait_queue,
				 (hdcp->hdcp_down_event & 0xFF) == 0);
	ret = (hdcp->hdcp_down_event & 0xFF00) >> 8;

	HDCP_DBG("User space task done %x\n", hdcp->hdcp_down_event);
	hdcp->hdcp_down_event = 0;

	return ret;
}

static int hdcp_wq_start_authentication(void)
{
	int status = 0;
	struct hdmi_ip_data *ip_data;

	HDCP_DBG("hdcp_wq_start_authentication %ums\n",
		jiffies_to_msecs(jiffies));

	if (hdmi_runtime_get()) {
		HDCP_ERR("%s Error enabling clocks\n", __func__);
		return -EINVAL;
	}

	ip_data = get_hdmi_ip_data();
	ip_data->ops->hdcp_enable(ip_data);

	hdmi_runtime_put();
	return status;
}

static void hdcp_work_queue(struct work_struct *work)
{
	struct hdcp_worker_data *d = container_of(work, typeof(*d), dwork.work);
	int state = atomic_read(&d->state);

	HDCP_DBG("hdcp_work_queue() start\n");
	switch (state) {
	case HDCP_STATE_STEP1:
		hdcp_wq_start_authentication();
	break;
	case HDCP_STATE_STEP2:
		hdcp_step2_authenticate_repeater(HDCP_EVENT_STEP2);
	break;
	}
	HDCP_DBG("hdcp_work_queue() - END\n");
}

static int hdcp_3des_load_key(uint32_t *deshdcp_encrypted_key)
{
	int counter = 0, status = HDCP_OK;

	if (!deshdcp_encrypted_key) {
		HDCP_ERR("HDCP keys NULL, failed to load keys\n");
		return HDCP_3DES_ERROR;
	}

	HDCP_DBG("Loading HDCP keys...\n");

	/* Set decryption mode in DES control register */
	WR_FIELD_32(hdcp->deshdcp_base_addr,
		DESHDCP__DHDCP_CTRL,
		DESHDCP__DHDCP_CTRL__DIRECTION_POS_F,
		DESHDCP__DHDCP_CTRL__DIRECTION_POS_L,
		0x0);

	/* Write encrypted data */
	while (counter < DESHDCP_KEY_SIZE) {
		/* Fill Data registers */
		WR_REG_32(hdcp->deshdcp_base_addr, DESHDCP__DHDCP_DATA_L,
			deshdcp_encrypted_key[counter]);
		WR_REG_32(hdcp->deshdcp_base_addr, DESHDCP__DHDCP_DATA_H,
			deshdcp_encrypted_key[counter + 1]);

		/* Wait for output bit at '1' */
		while (RD_FIELD_32(hdcp->deshdcp_base_addr,
			DESHDCP__DHDCP_CTRL,
			DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_F,
			DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_L) != 0x1)
			;

		/* Dummy read (indeed data are transfered directly into
		 * key memory)
		 */
		if (RD_REG_32(hdcp->deshdcp_base_addr, DESHDCP__DHDCP_DATA_L)
				!= 0x0) {
			status = -HDCP_3DES_ERROR;
			HDCP_ERR("DESHDCP dummy read error\n");
		}
		if (RD_REG_32(hdcp->deshdcp_base_addr, DESHDCP__DHDCP_DATA_H) !=
			0x0) {
			status = -HDCP_3DES_ERROR;
			HDCP_ERR("DESHDCP dummy read error\n");
		}

		counter += 2;
	}

	if (status == HDCP_OK)
		hdcp->hdcp_keys_loaded = true;

	return status;
}

static bool hdcp_3des_cb(void)
{
	HDCP_DBG("hdcp_3des_cb() %u\n", jiffies_to_msecs(jiffies));

	if (!hdcp->hdcp_keys_loaded) {
		HDCP_ERR("%s: hdcp_keys not loaded = %d\n",
				__func__, hdcp->hdcp_keys_loaded);
		return false;
	}

	/* Load 3DES key */
	if (hdcp_3des_load_key(hdcp->en_ctrl->key) != HDCP_OK) {
		HDCP_ERR("Error Loading  HDCP keys\n");
		return false;
	}

	return true;
}

static void hdcp_start_frame_cb(void)
{
	HDCP_DBG("hdcp_start_frame_cb() %ums\n", jiffies_to_msecs(jiffies));

	if (!hdcp->hdcp_keys_loaded) {
		HDCP_DBG("%s: hdcp_keys not loaded = %d\n",
		    __func__, hdcp->hdcp_keys_loaded);
		return;
	}

	/* Cancel any previous work submitted */
	__cancel_delayed_work(&hdcp->hdcp_work->dwork);
	atomic_set(&hdcp->hdcp_work->state, HDCP_STATE_STEP1);
	/* HDCP enable after 7 Vsync delay */
	queue_delayed_work(hdcp->workqueue, &hdcp->hdcp_work->dwork,
				msecs_to_jiffies(300));

}

static long hdcp_query_status_ctl(uint32_t *status)
{
	struct hdmi_ip_data *ip_data;

	HDCP_DBG("hdcp_ioctl() - QUERY %u", jiffies_to_msecs(jiffies));

	ip_data = get_hdmi_ip_data();
	if (!ip_data) {
		HDCP_ERR("null pointer hit\n");
		return -EINVAL;
	}

	*status = ip_data->ops->hdcp_status(ip_data);

	return 0;
}

static long hdcp_auth_wait_event_ctl(struct hdcp_wait_control *ctrl)
{
	HDCP_DBG("hdcp_ioctl() - WAIT %u %d", jiffies_to_msecs(jiffies),
					 hdcp->hdcp_up_event);

	wait_event_interruptible(hdcp_up_wait_queue,
				 (hdcp->hdcp_up_event & 0xFF) != 0);

	ctrl->event = hdcp->hdcp_up_event;

	hdcp->hdcp_up_event = 0;

	return 0;
}

static long hdcp_user_space_done_ctl(uint32_t *status)
{
	HDCP_DBG("hdcp_ioctl() - DONE %u %d", jiffies_to_msecs(jiffies),
		*status);

	hdcp->hdcp_down_event &= ~(*status & 0xFF);
	hdcp->hdcp_down_event |= *status & 0xFF00;

	wake_up_interruptible(&hdcp_down_wait_queue);

	return 0;
}

static long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct hdcp_wait_control ctrl;
	uint32_t status;

	if (!fd) {
		HDCP_ERR("%s null pointer\n", __func__);
		return -EINVAL;
	}

	HDCP_DBG("%s\n", __func__);

	switch (cmd) {
	case HDCP_WAIT_EVENT:

		if (copy_from_user(&ctrl, argp,
				sizeof(struct hdcp_wait_control))) {
			HDCP_ERR("HDCP: Error copying from user space"
				    " - wait ioctl");
			return -EFAULT;
		}

		/* Do not allow re-entrance for this ioctl */
		mutex_lock(&hdcp->re_entrant_lock);
		if (hdcp->re_entrance_flag == false) {
			hdcp->re_entrance_flag = true;
			mutex_unlock(&hdcp->re_entrant_lock);

			hdcp_auth_wait_event_ctl(&ctrl);

			mutex_lock(&hdcp->re_entrant_lock);
			hdcp->re_entrance_flag = false;
			mutex_unlock(&hdcp->re_entrant_lock);
		} else {
			mutex_unlock(&hdcp->re_entrant_lock);
			ctrl.event = HDCP_EVENT_EXIT;
		}

		/* Store output data to output pointer */
		if (copy_to_user(argp, &ctrl,
			 sizeof(struct hdcp_wait_control))) {
			HDCP_ERR("HDCP: Error copying to user space -"
				    " wait ioctl");
			return -EFAULT;
		}
		break;

	case HDCP_QUERY_STATUS:

		hdcp_query_status_ctl(&status);

		/* Store output data to output pointer */
		if (copy_to_user(argp, &status,
			sizeof(uint32_t))) {
			HDCP_ERR("HDCP: Error copying to user space -"
				"query status ioctl");
			return -EFAULT;
		}

		break;

	case HDCP_DONE:
		if (copy_from_user(&status, argp,
				sizeof(uint32_t))) {
			HDCP_ERR("HDCP: Error copying from user space"
				    " - done ioctl");
			return -EFAULT;
		}

		return hdcp_user_space_done_ctl(&status);

	default:
		return -ENOTTY;
	} /* End switch */

	return 0;
}

static int hdcp_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int status;

	HDCP_DBG("hdcp_mmap() %lx %lx %lx\n", vma->vm_start, vma->vm_pgoff,
					 vma->vm_end - vma->vm_start);

	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	status = remap_pfn_range(vma, vma->vm_start,
				 HDMI_CORE >> PAGE_SHIFT,
				 vma->vm_end - vma->vm_start,
				 vma->vm_page_prot);
	if (status) {
		HDCP_DBG("mmap error %d\n", status);
		return -EAGAIN;
	}

	HDCP_DBG("mmap succesful\n");
	return 0;
}

static void hdcp_load_keys_cb(const struct firmware *fw, void *context)
{
	struct hdcp_enable_control *en_ctrl;

	HDCP_DBG("%s\n", __func__);

	if (!fw) {
		HDCP_ERR("failed to load keys\n");
		return;
	}

	if (fw->size != sizeof(en_ctrl->key)) {
		HDCP_ERR("encrypted key file wrong size %d\n", fw->size);
		return;
	}

	en_ctrl = kzalloc(sizeof(*en_ctrl), GFP_KERNEL);
	if (!en_ctrl) {
		HDCP_ERR("can't allocated space for keys\n");
		return;
	}

	memcpy(en_ctrl->key, fw->data, sizeof(en_ctrl->key));
	en_ctrl->nb_retry = 20;

	hdcp->en_ctrl = en_ctrl;
	hdcp->hdcp_keys_loaded = true;
	HDCP_INFO("loaded keys\n");
}

static int hdcp_load_keys(void)
{
	int ret;

	HDCP_DBG("%s\n", __func__);

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
					"hdcp.keys", hdcp->mdev->this_device,
					GFP_KERNEL, hdcp, hdcp_load_keys_cb);
	if (ret < 0) {
		HDCP_ERR("request_firmware_nowait failed: %d\n", ret);
		hdcp->hdcp_keys_loaded = false;
	}

	return ret;
}

static void hdcp_irq_cb(void)
{
	struct hdmi_ip_data *ip_data;
	u32 intr = 0;

	ip_data = get_hdmi_ip_data();

	if (ip_data->ops->hdcp_int_handler)
		intr = ip_data->ops->hdcp_int_handler(ip_data);

	if (intr == KSVACCESSINT) {
		__cancel_delayed_work(&hdcp->hdcp_work->dwork);
		atomic_set(&hdcp->hdcp_work->state, HDCP_STATE_STEP2);
		queue_delayed_work(hdcp->workqueue, &hdcp->hdcp_work->dwork, 0);
	}

	return;
}


static const struct file_operations hdcp_fops = {
	.owner = THIS_MODULE,
	.mmap = hdcp_mmap,
	.unlocked_ioctl = hdcp_ioctl,
};

static int __init hdcp_init(void)
{
	HDCP_DBG("hdcp_init() %ums\n", jiffies_to_msecs(jiffies));

	/* Allocate HDCP struct */
	hdcp = kzalloc(sizeof(struct hdcp_data), GFP_KERNEL);
	if (!hdcp) {
		HDCP_ERR("Could not allocate HDCP structure\n");
		return -ENOMEM;
	}

	hdcp->hdcp_work = kzalloc(sizeof(struct hdcp_worker_data), GFP_KERNEL);
	if (!hdcp->hdcp_work) {
		HDCP_ERR("Could not allocate HDCP worker  structure\n");
		goto err_alloc_worker;
	}

	hdcp->mdev = kzalloc(sizeof(struct miscdevice), GFP_KERNEL);
	if (!hdcp->mdev) {
		HDCP_ERR("Could not allocate misc device memory\n");
		goto err_alloc_mdev;
	}

	/* Map DESHDCP in kernel address space */
	hdcp->deshdcp_base_addr = ioremap(DSS_SS_FROM_L3__DESHDCP, 0x34);

	if (!hdcp->deshdcp_base_addr) {
		HDCP_ERR("DESHDCP IOremap error\n");
		goto err_map_deshdcp;
	}

	mutex_init(&hdcp->lock);

	hdcp->mdev->minor = MISC_DYNAMIC_MINOR;
	hdcp->mdev->name = "hdcp";
	hdcp->mdev->mode = 0666;
	hdcp->mdev->fops = &hdcp_fops;

	if (misc_register(hdcp->mdev)) {
		HDCP_ERR("Could not add character driver\n");
		goto err_misc_register;
	}

	hdcp->workqueue = create_singlethread_workqueue("hdcp");
	if (hdcp->workqueue == NULL) {
		HDCP_ERR("Could not create HDCP workqueue\n");
		goto err_add_driver;
	}

	INIT_DELAYED_WORK(&hdcp->hdcp_work->dwork, hdcp_work_queue);

	mutex_init(&hdcp->re_entrant_lock);

	if (hdmi_runtime_get()) {
		HDCP_ERR("%s Error enabling clocks\n", __func__);
		goto err_runtime;
	}

	/* Register HDCP callbacks to HDMI library */
	omapdss_hdmi_register_hdcp_callbacks(&hdcp_start_frame_cb,
					 &hdcp_3des_cb, &hdcp_irq_cb);

	hdmi_runtime_put();

	hdcp_load_keys();

	return 0;

err_runtime:
	mutex_destroy(&hdcp->re_entrant_lock);

	destroy_workqueue(hdcp->workqueue);

err_add_driver:
	misc_deregister(hdcp->mdev);

err_misc_register:
	mutex_destroy(&hdcp->lock);

	iounmap(hdcp->deshdcp_base_addr);

err_map_deshdcp:
	kfree(hdcp->mdev);

err_alloc_mdev:
	kfree(hdcp->hdcp_work);

err_alloc_worker:
	kfree(hdcp);
	return -EFAULT;
}

static void __exit hdcp_exit(void)
{
	HDCP_DBG("hdcp_exit() %ums\n", jiffies_to_msecs(jiffies));

	mutex_lock(&hdcp->lock);

	kfree(hdcp->en_ctrl);

	if (hdmi_runtime_get()) {
		HDCP_ERR("%s Error enabling clocks\n", __func__);
		goto err_handling;
	}

	/* Un-register HDCP callbacks to HDMI library */
	omapdss_hdmi_register_hdcp_callbacks(NULL, NULL, NULL);

	hdmi_runtime_put();

err_handling:
	misc_deregister(hdcp->mdev);
	kfree(hdcp->mdev);

	iounmap(hdcp->deshdcp_base_addr);

	kfree(hdcp->hdcp_work);
	destroy_workqueue(hdcp->workqueue);

	mutex_unlock(&hdcp->lock);

	mutex_destroy(&hdcp->lock);

	mutex_destroy(&hdcp->re_entrant_lock);

	kfree(hdcp);
}

module_init(hdcp_init);
module_exit(hdcp_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OMAP HDCP kernel module");
MODULE_AUTHOR("Sujeet Baranwal");
