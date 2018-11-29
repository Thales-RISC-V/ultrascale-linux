/*
 *  risc-v-ft.c - Fault tolerant Risc-V controller
 *
 *  Copyright (C) Antmicro Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <linux/delay.h>

#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>

#include "risc-v-ft.h"

static struct risc_v_private *priv_data;

static void risc_v_write_reg(struct risc_v_private *priv, uint32_t offset,
							uint32_t value)
{
	uint32_t *reg = (uint32_t*)(priv->base_addr + offset);
	*reg = value;
}

static uint32_t risc_v_read_reg(struct risc_v_private *priv, uint32_t offset)
{
	uint32_t *reg = (uint32_t*)(priv->base_addr + offset);
	return *reg;
}

static irqreturn_t risc_v_irq_handler(int irq, void *data)
{
	struct risc_v_private *priv = (struct risc_v_private *)data;
	uint32_t reg;
	uint32_t jtag_routing;
	int32_t faulty_cpu = -1;

	reg = risc_v_read_reg(priv, RISC_V_STATUS_REG);

	/* check the interrupt cause */
	if(reg & RISC_V_FAULT_IRQ) {

		switch(~(reg) & RISC_V_FAULT_MASK)
		{
			case RISC_V_CPU0_FAULT:
				jtag_routing = RISC_V_JTAG0_CPU0;
				faulty_cpu = 0;
				break;
			case RISC_V_CPU1_FAULT:
				jtag_routing = RISC_V_JTAG0_CPU1;
				faulty_cpu = 1;
				break;
			case RISC_V_CPU2_FAULT:
				jtag_routing = RISC_V_JTAG0_CPU2;
				faulty_cpu = 2;
				break;
			default:
				jtag_routing = 0;
				faulty_cpu = -1;
		}

		printk(KERN_ERR"Detected fault on CPU %d !!!!!\n", faulty_cpu);

		/* mask the fault irq and enable online irq*/
		reg = risc_v_read_reg(priv, RISC_V_CTRL_REG);
		reg &= ~(RISC_V_FAULT_INTERRUPT_ENABLE);
		reg |= RISC_V_CPU_ONLINE_INTERRUPT_ENABLE;
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);

		/* route JTAG to the faulty CPU */
		reg |= (jtag_routing << RISC_V_JTAG_ROUTING_OFFSET);
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);

		/* ack the interrupt */
		risc_v_write_reg(priv, RISC_V_STATUS_REG, RISC_V_FAULT_IRQ);

	} else if (reg & RISC_V_CPU_ONLINE_IRQ) {

		printk(KERN_ERR"CPU is back online !!!\n");
		/* mask the online and enable fault irq */
		reg = risc_v_read_reg(priv, RISC_V_CTRL_REG);
		reg |= RISC_V_FAULT_INTERRUPT_ENABLE;
		reg &= ~(RISC_V_CPU_ONLINE_INTERRUPT_ENABLE);
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);

		/* reset JTAG routing */
		reg &= ~(RISC_V_JTAG_MASK << RISC_V_JTAG_ROUTING_OFFSET);
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);

		/* ack the interrupt */
		risc_v_write_reg(priv, RISC_V_STATUS_REG, RISC_V_CPU_ONLINE_IRQ);
	}

	return IRQ_HANDLED;
}

static int risc_v_open(struct inode *inode, struct file *filp)
{
	filp->private_data = priv_data;
	return 0;
}

static int risc_v_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static long risc_v_ioctl(struct file *filp, unsigned int ioctl_num,
						unsigned long ioctl_param)
{
	struct risc_v_private *priv = filp->private_data;
	unsigned int reg;
	unsigned int status;
	unsigned int cpu_number;

	unsigned int timeout;
	unsigned int firmware_type;

	switch (ioctl_num) {
	case RISC_V_RESET_CPUS:
		reg = risc_v_read_reg(priv, RISC_V_CTRL_REG);
		reg |= RISC_V_RESET_ROUTING;
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);
		reg |= RISC_V_RESET_ALL;
		reg &= ~(RISC_V_INTERRUPT_ENABLE);
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);
		break;
	case RISC_V_RELEASE_RESET:
		reg = risc_v_read_reg(priv, RISC_V_CTRL_REG);
		reg &= ~(RISC_V_RESET_ALL);
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);

		risc_v_write_reg(priv, RISC_V_STATUS_REG, RISC_V_CPU_ONLINE_IRQ);
		timeout = 100;
		do{

			status = risc_v_read_reg(priv, RISC_V_STATUS_REG);
			udelay(100);
			if(!(--timeout)) break;

		}while(!(status & RISC_V_CPU_ONLINE_IRQ));

		risc_v_write_reg(priv, RISC_V_STATUS_REG, RISC_V_CPU_ONLINE_IRQ);

		/* clear any pending interrupt interrupt */
		risc_v_write_reg(priv, RISC_V_STATUS_REG, RISC_V_FAULT_IRQ);
		/* release the routing reset */
		reg |= RISC_V_INTERRUPT_ENABLE | RISC_V_FAULT_INTERRUPT_ENABLE;
		reg &= ~(RISC_V_RESET_ROUTING);
		reg &= ~(RISC_V_JTAG_MASK << RISC_V_JTAG_ROUTING_OFFSET);
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);
		break;
	case RISC_V_ENABLE_CLKS:
		reg = risc_v_read_reg(priv, RISC_V_CTRL_REG);
		reg |= RISC_V_ENABLE_ALL;
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);
		break;
	case RISC_V_DISABLE_CLKS:
		reg = risc_v_read_reg(priv, RISC_V_CTRL_REG);
		reg &= ~(RISC_V_ENABLE_ALL);
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);
		break;
	case RISC_V_RESET_SINGLE:
		if(copy_from_user(&cpu_number, (void*)ioctl_param, sizeof(unsigned int)))
			return -EFAULT;
		reg = risc_v_read_reg(priv, RISC_V_CTRL_REG);

		/* reset the CPU */
		reg |= RISC_V_RESET_CPU0 << cpu_number;
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);

		/* enable the CPU again */
		reg &= ~(RISC_V_RESET_CPU0 << cpu_number);
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);
		break;
	case RISC_V_BREAK_CPU:
		if(copy_from_user(&cpu_number, (void*)ioctl_param, sizeof(unsigned int)))
			return -EFAULT;
		reg = risc_v_read_reg(priv, RISC_V_CTRL_REG);

		/* disable the CPU */
		reg &= ~(1<<cpu_number);
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);

		/* enable the CPU again */
		reg |= 1<<cpu_number;
		risc_v_write_reg(priv, RISC_V_CTRL_REG, reg);
		break;
	case RISC_V_SET_FIRMWARE_TYPE:
		if(copy_from_user(&firmware_type, (void*)ioctl_param, sizeof(unsigned int)))
			return -EFAULT;
		priv->recovery = firmware_type;
		break;

	default:
		return -EINVAL;
	}

	return 0;

}

static ssize_t risc_v_write(struct file *filp, const char __user *data,
			      size_t len, loff_t *ppose)
{
	//uint32_t reg;
	int err;
	int i;
	struct risc_v_private *priv = filp->private_data;
	//TODO: validate the firmware (maybe we should load ELFs?)

	printk("FIRMWARE TYPE %d\n", priv->recovery);
	if (priv->recovery) {
		/* load the recovery firmware */
		for (i = 0; i < 3; i++) {
			err = copy_from_user(priv->recovery_buffer[i], data, len);
		}
	} else {
		/* load the firmware */
		err = copy_from_user(priv->firmware_buffer, data, len);
	}
	if(err) {
		dev_err(&priv->dev, "Failed to copy the firmware\n");
		return err;
	}
	return len;
}


static const struct file_operations risc_v_fops = {
	.owner		= THIS_MODULE,
	.write		= risc_v_write,
	.open		= risc_v_open,
	.unlocked_ioctl = risc_v_ioctl,
	.release	= risc_v_release,
};

static struct miscdevice risc_v_driver = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "risc-v-ft",
	.fops		= &risc_v_fops
};


static int risc_v_probe(struct platform_device *pdev)
{
	struct risc_v_private *private;
	struct device_node *firmware_node;
	struct resource *res;
	int err = 0;

	private = devm_kzalloc(&pdev->dev, sizeof(*private), GFP_KERNEL);
	if(!private)
			return -ENOMEM;

	firmware_node = of_parse_phandle(pdev->dev.of_node, "risc-v-memory", 0);
	if(!firmware_node) {
		dev_err(&pdev->dev, "Failed to get the firmware node\n");
		return -EINVAL;
	}

	err = of_address_to_resource(firmware_node, 0, &private->firmware_phys);
	if(err) {
		dev_err(&pdev->dev, "Failed to get firmware mem space \n");
		return err;
	}

	private->firmware_buffer = devm_ioremap_nocache(&pdev->dev,
						private->firmware_phys.start,
						private->firmware_phys.end - private->firmware_phys.start);

	private->recovery_buffer[0] = devm_ioremap_nocache(&pdev->dev, RISC_V_RECOVERY_MEM0, 0x1000);
	private->recovery_buffer[1] = devm_ioremap_nocache(&pdev->dev, RISC_V_RECOVERY_MEM1, 0x1000);
	private->recovery_buffer[2] = devm_ioremap_nocache(&pdev->dev, RISC_V_RECOVERY_MEM2, 0x1000);

	//Write to firmware memory by default
	private->recovery = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	private->base_addr = devm_ioremap_resource(&pdev->dev, res);

	if(!private->base_addr) {
		dev_err(&pdev->dev, "Failed to map register space \n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, private);
	private->dev = pdev->dev;

	/* set global private pointer */
	priv_data = private;

	private->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	err = devm_request_irq(&pdev->dev, private->irq, risc_v_irq_handler, IRQF_SHARED, "risc-v-ft",
						private);
	if(err) {
		dev_err(&pdev->dev, "Failed to register interrupt [err = %d]\n", err);
		return err;
	}

	err = misc_register(&risc_v_driver);
	if(err)
		dev_err(&pdev->dev, "Failed to register misc device [err = %d]\n", err);

	return err;
}

static int risc_v_remove(struct platform_device *pdev)
{
	misc_deregister(&risc_v_driver);
	return 0;
}


static const struct of_device_id risc_v_of_match[] = {
	{ .compatible = "antmicro,risc-v-ft", },
	{}
};

MODULE_DEVICE_TABLE(of, risc_v_of_match);

static struct platform_driver risc_v_platform_driver = {
	.probe		= risc_v_probe,
	.remove		= risc_v_remove,
	.driver		=  {
				.owner = THIS_MODULE,
				.name = "Risc-V FT manager",
				.of_match_table = risc_v_of_match,
	},
};

static int __init risc_v_init(void)
{
	return platform_driver_register(&risc_v_platform_driver);
}

static void __exit risc_v_exit(void)
{
	platform_driver_unregister(&risc_v_platform_driver);
}

module_init(risc_v_init);
module_exit(risc_v_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Karol Gugala");
MODULE_DESCRIPTION("Risc-V FT driver");

