#ifndef _RISC_V_FT_H_
#define _RISC_V_FT_H_

#include <uapi/misc/risc-v-ft.h>

struct risc_v_private {
	struct resource firmware_phys;
	void __iomem *base_addr;
	void __iomem *firmware_buffer;
	void __iomem *recovery_buffer[3];

	int irq;
	int recovery; //0 - program, 1 - recovery

	struct device dev;
};

/* CTRL register */
#define RISC_V_CTRL_REG (0x00)

#define RISC_V_ENABLE_CPU0 (1<<0)
#define RISC_V_ENABLE_CPU1 (1<<1)
#define RISC_V_ENABLE_CPU2 (1<<2)

#define RISC_V_ENABLE_ALL (RISC_V_ENABLE_CPU0 | RISC_V_ENABLE_CPU1 | RISC_V_ENABLE_CPU2)

#define RISC_V_RESET_CPU0 (1<<4)
#define RISC_V_RESET_CPU1 (1<<5)
#define RISC_V_RESET_CPU2 (1<<6)
#define RISC_V_RESET_ALL (RISC_V_RESET_CPU0 | RISC_V_RESET_CPU1 | RISC_V_RESET_CPU2)

#define RISC_V_INTERRUPT_ENABLE (1<<8)
#define RISC_V_FAULT_INTERRUPT_ENABLE (1<<9)
#define RISC_V_CPU_ONLINE_INTERRUPT_ENABLE (1<<10)

#define RISC_V_JTAG0_CPU0 (1)
#define RISC_V_JTAG0_CPU1 (2)
#define RISC_V_JTAG0_CPU2 (3)
#define RISC_V_JTAG_MASK 0x7

#define RISC_V_JTAG_ROUTING_OFFSET (12)

#define RISC_V_RESET_ROUTING_CPU0 (1 << 20)
#define RISC_V_RESET_ROUTING_CPU1 (1 << 21)
#define RISC_V_RESET_ROUTING_CPU2 (1 << 22)

#define RISC_V_RESET_ROUTING (RISC_V_RESET_ROUTING_CPU0 | RISC_V_RESET_ROUTING_CPU1 | RISC_V_RESET_ROUTING_CPU2)

/* status reg */
#define RISC_V_STATUS_REG (0x04)

#define RISC_V_FAULT_IRQ (1 << 0)
#define RISC_V_CPU_ONLINE_IRQ (1 << 1)

#define RISC_V_CPU0_FAULT (1 << 4)
#define RISC_V_CPU1_FAULT (1 << 5)
#define RISC_V_CPU2_FAULT (1 << 6)

#define RISC_V_FAULT_SHIFT 4
#define RISC_V_FAULT_MASK (0x7 << RISC_V_FAULT_SHIFT)

#define RISC_V_RECOVERY_MEM0 0xA0000000
#define RISC_V_RECOVERY_MEM1 0xA0001000
#define RISC_V_RECOVERY_MEM2 0xA0002000

#endif
