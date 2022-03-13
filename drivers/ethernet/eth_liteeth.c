/*
 * Copyright (c) 2019 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT litex_eth0

#define LOG_MODULE_NAME eth_liteeth
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <stdbool.h>
#include <net/ethernet.h>
#include <net/net_if.h>
#include <net/net_pkt.h>

#include <sys/printk.h>

#include "eth.h"

/* flags */
#define LITEETH_EV_TX		0x1
#define LITEETH_EV_RX		0x1

/* slots */
#define LITEETH_SLOT_BASE	DT_INST_REG_ADDR_BY_NAME(0, buffers)
#define LITEETH_SLOT_RX0	((LITEETH_SLOT_BASE) + 0x0000)
#define LITEETH_SLOT_RX1	((LITEETH_SLOT_BASE) + 0x0800)
#define LITEETH_SLOT_TX0	((LITEETH_SLOT_BASE) + 0x1000)
#define LITEETH_SLOT_TX1	((LITEETH_SLOT_BASE) + 0x1800)

/* sram - rx */
#define LITEETH_RX_BASE		DT_INST_REG_ADDR_BY_NAME(0, control)
#define LITEETH_RX_SLOT		((LITEETH_RX_BASE) + 0x00)
#define LITEETH_RX_LENGTH	((LITEETH_RX_BASE) + 0x04)
#define LITEETH_RX_EV_PENDING	((LITEETH_RX_BASE) + 0x10)
#define LITEETH_RX_EV_ENABLE	((LITEETH_RX_BASE) + 0x14)

/* sram - tx */
#define LITEETH_TX_BASE		((DT_INST_REG_ADDR_BY_NAME(0, control)) + 0x18)
#define LITEETH_TX_START	((LITEETH_TX_BASE) + 0x00)
#define LITEETH_TX_READY	((LITEETH_TX_BASE) + 0x04)
#define LITEETH_TX_SLOT		((LITEETH_TX_BASE) + 0x0c)
#define LITEETH_TX_LENGTH	((LITEETH_TX_BASE) + 0x10)
#define LITEETH_TX_EV_PENDING	((LITEETH_TX_BASE) + 0x18)

/* irq */
#define LITEETH_IRQ		DT_INST_IRQN(0)
#define LITEETH_IRQ_PRIORITY	DT_INST_IRQ(0, priority)

#define MAX_TX_FAILURE 100
extern bool halt_net_tx;
struct eth_liteeth_dev_data {
	struct net_if *iface;
	uint8_t mac_addr[6];

	uint8_t txslot;
	uint8_t rxslot;

	uint8_t *tx_buf[2];
	uint8_t *rx_buf[2];
};

struct eth_liteeth_config {
	void (*config_func)(void);
};

static int eth_initialize(const struct device *dev)
{
	const struct eth_liteeth_config *config = dev->config;

	config->config_func();

	return 0;
}

static int eth_tx(const struct device *dev, struct net_pkt *pkt)
{
	if(halt_net_tx) { return 0; }
	int key;
	uint16_t len;
	struct eth_liteeth_dev_data *context = dev->data;

	key = irq_lock();
	int attempts = 0;

	/* get data from packet and send it */
	len = net_pkt_get_len(pkt);
	net_pkt_read(pkt, context->tx_buf[context->txslot], len);

	uint32_t* slot_base = (uint32_t *)(context->tx_buf[context->txslot]);
	uint32_t magic = *(slot_base + 9);
// 	LOG_ERR("Looking at %ul for magic: %ul\n",(uint32_t)slot_base,magic);
// 	LOG_ERR("Looking at 0x%08x for magic: 0x%08x\n",(uint32_t)slot_base,magic);
	//Magic is the destination port (1337) and length
	if(magic == 0xce033905) {
		LOG_INF("FPGA Configuration Packet found, setting up FPGA Tx Slots");
		//Zero out checksum
		*(context->tx_buf[context->txslot]+40) = 0x00;
		*(context->tx_buf[context->txslot]+41) = 0x00;
		memcpy((uint32_t*)(0x80002000 + 0x000), context->tx_buf[context->txslot], 42);
		memcpy((uint32_t*)(0x80002000 + 0x800), context->tx_buf[context->txslot], 42);
	}

// 	uint32_t* ctrl_reg = (uint32_t*)0xf0002800;
// 	uint32_t reg_before = *ctrl_reg;
// 	*ctrl_reg = 0;
	uint32_t* rgb_start = (uint32_t*)0xf0002808;
	uint32_t* rgb_slot = (uint32_t*)0xf000280c;
	uint32_t* rgb_length = (uint32_t*)0xf0002810;

	while (sys_read32(LITEETH_TX_READY) == 0) {
// 		if (attempts++ == MAX_TX_FAILURE) {
			goto error;
// 		}
		//k_sleep(K_MSEC(1));
	}
	*rgb_slot = context->txslot;
	*rgb_length = len;
	*rgb_start = 1;
// 	sys_write8(context->txslot, LITEETH_TX_SLOT);
// 	sys_write32(len, LITEETH_TX_LENGTH);

	//Read this a fiew times??
// 	for(int i = 0; i < 10; i++) {
// 		sys_read8(LITEETH_TX_READY);
// 	}
	/* wait for the device to be ready to transmit */
// 	while (sys_read32(LITEETH_TX_READY) == 0) {
// 		if (attempts++ == MAX_TX_FAILURE) {
// 			goto error;
// 		}
		//k_sleep(K_MSEC(1));
// 	}

	/* start transmitting */
// 	sys_write8(1, LITEETH_TX_START);
// 	while (sys_read32(LITEETH_TX_READY) == 0) {
// 		LOG_ERR("TX fifo nailed");
// 	}
// 	*ctrl_reg = reg_before;

	/* change slot */
	context->txslot = (context->txslot + 1) % 2;

	irq_unlock(key);
//  	printk("ok\n");

	return 0;
error:
	irq_unlock(key);
	LOG_ERR("TX fifo failed");
	return -1;
}

static void eth_rx(const struct device *port)
{
	struct net_pkt *pkt;
	struct eth_liteeth_dev_data *context = port->data;

	int r;
	unsigned int key;
	uint16_t len = 0;

	key = irq_lock();
	len = sys_read32(LITEETH_RX_LENGTH);

	/* which slot is the frame in */
	context->rxslot = sys_read8(LITEETH_RX_SLOT);

	/* obtain rx buffer */
	pkt = net_pkt_rx_alloc_with_buffer(context->iface, len, AF_UNSPEC, 0,
					   K_NO_WAIT);
	if (pkt == NULL) {
		LOG_ERR("Failed to obtain RX buffer len: %d",len);
		goto out;
	}

	/* copy data to buffer */
	if (net_pkt_write(pkt, (void *)context->rx_buf[context->rxslot], len) != 0) {
		LOG_ERR("Failed to append RX buffer to context buffer");
		net_pkt_unref(pkt);
		goto out;
	}

	/* receive data */
	r = net_recv_data(context->iface, pkt);
	if (r < 0) {
		LOG_ERR("Failed to enqueue frame into RX queue: %d", r);
		net_pkt_unref(pkt);
	}

out:
	irq_unlock(key);
}

static void eth_irq_handler(const struct device *port)
{
	/* check sram reader events (tx) */
	if (sys_read8(LITEETH_TX_EV_PENDING) & LITEETH_EV_TX) {
		/* TX event is not enabled nor used by this driver; ack just
		 * in case if some rogue TX event appeared
		 */
		sys_write8(LITEETH_EV_TX, LITEETH_TX_EV_PENDING);
	}

	/* check sram writer events (rx) */
	if (sys_read8(LITEETH_RX_EV_PENDING) & LITEETH_EV_RX) {
		eth_rx(port);

		/* ack writer irq */
		sys_write8(LITEETH_EV_RX, LITEETH_RX_EV_PENDING);
	}
}

#ifdef CONFIG_ETH_LITEETH_0

static struct eth_liteeth_dev_data eth_data = {
	.mac_addr =  DT_INST_PROP(0, local_mac_address)
};

static void eth_irq_config(void);
static const struct eth_liteeth_config eth_config = {
	.config_func = eth_irq_config,
};

static void eth_iface_init(struct net_if *iface)
{
	const struct device *port = net_if_get_device(iface);
	struct eth_liteeth_dev_data *context = port->data;
	static bool init_done;

	/* initialize only once */
	if (init_done) {
		return;
	}

	/* set interface */
	context->iface = iface;

	/* initialize ethernet L2 */
	ethernet_init(iface);

#if DT_INST_PROP(0, zephyr_random_mac_address)
	/* generate random MAC address */
	gen_random_mac(context->mac_addr, 0x10, 0xe2, 0xd5);
#endif

	/* set MAC address */
	if (net_if_set_link_addr(iface, context->mac_addr, sizeof(context->mac_addr),
			     NET_LINK_ETHERNET) < 0) {
		LOG_ERR("setting mac failed");
		return;
	}

	/* clear pending events */
	sys_write8(LITEETH_EV_TX, LITEETH_TX_EV_PENDING);
	sys_write8(LITEETH_EV_RX, LITEETH_RX_EV_PENDING);

	/* setup tx slots */
	context->txslot = 0;
	context->tx_buf[0] = (uint8_t *)LITEETH_SLOT_TX0;
	context->tx_buf[1] = (uint8_t *)LITEETH_SLOT_TX1;

	/* setup rx slots */
	context->rxslot = 0;
	context->rx_buf[0] = (uint8_t *)LITEETH_SLOT_RX0;
	context->rx_buf[1] = (uint8_t *)LITEETH_SLOT_RX1;

	init_done = true;
}

static enum ethernet_hw_caps eth_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T |
	       ETHERNET_LINK_1000BASE_T;
}

static const struct ethernet_api eth_api = {
	.iface_api.init = eth_iface_init,
	.get_capabilities = eth_caps,
	.send = eth_tx
};

NET_DEVICE_DT_INST_DEFINE(0, eth_initialize, NULL,
		&eth_data, &eth_config, CONFIG_ETH_INIT_PRIORITY, &eth_api,
		ETHERNET_L2, NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);

static void eth_irq_config(void)
{
	IRQ_CONNECT(LITEETH_IRQ, LITEETH_IRQ_PRIORITY, eth_irq_handler,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(LITEETH_IRQ);
	sys_write8(1, LITEETH_RX_EV_ENABLE);
}

#endif /* CONFIG_ETH_LITEETH_0 */
