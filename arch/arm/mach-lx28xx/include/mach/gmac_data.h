
#ifndef _GMAC_DATA_H__
#define _GMAC_DATA_H__

#define MTL_MAX_RX_QUEUES	8
#define MTL_MAX_TX_QUEUES	8

struct plat_mdio_bus_data {
	int (*phy_reset)(void *priv);
	unsigned int phy_mask;
	int *irqs;
	int probed_phy_irq;
#ifdef CONFIG_OF
	int reset_gpio, active_low;
	u32 delays[3];
#endif
};

struct stmmac_rxq_cfg {
	u8 mode_to_use;
	u32 chan;
	u8 pkt_route;
	bool use_prio;
	u32 prio;
};

struct stmmac_txq_cfg {
	u32 weight;
	u8 mode_to_use;
	/* Credit Base Shaper parameters */
	u32 send_slope;
	u32 idle_slope;
	u32 high_credit;
	u32 low_credit;
	bool use_prio;
	u32 prio;
};

struct stmmac_dma_cfg {
	int pbl;
	int txpbl;
	int rxpbl;
	bool pblx8;
	int fixed_burst;
	int mixed_burst;
	bool aal;
};


/* Private data for the STM on-board ethernet driver */
struct plat_gmac_data {
#if 0
    int bus_id;
    int pbl;
    int clk_csr;
    int has_gmac;
    int enh_desc;
    int tx_coe;
    int bugged_jumbo;
    int pmt;
    void (*fix_mac_speed)(void *priv, unsigned int speed);
    void (*bus_setup)(void __iomem *ioaddr);
    int (*init)(struct platform_device *pdev);
    void (*exit)(struct platform_device *pdev);
    void *custom_cfg;
    void *bsp_priv;
#endif
	int bus_id;
	int phy_addr;
	int interface;
	struct stmmac_mdio_bus_data *mdio_bus_data;
	struct device_node *phy_node;
	struct device_node *mdio_node;
	struct stmmac_dma_cfg *dma_cfg;
	int clk_csr;
	int has_gmac;
	int enh_desc;
	int tx_coe;
	int rx_coe;
	int bugged_jumbo;
	int pmt;
	int force_sf_dma_mode;
	int force_thresh_dma_mode;
	int riwt_off;
	int max_speed;
	int maxmtu;
	int multicast_filter_bins;
	int unicast_filter_entries;
	int tx_fifo_size;
	int rx_fifo_size;
	u32 rx_queues_to_use;
	u32 tx_queues_to_use;
	u8 rx_sched_algorithm;
	u8 tx_sched_algorithm;
	struct stmmac_rxq_cfg rx_queues_cfg[MTL_MAX_RX_QUEUES];
	struct stmmac_txq_cfg tx_queues_cfg[MTL_MAX_TX_QUEUES];
	void (*fix_mac_speed)(void *priv, unsigned int speed);
	int (*init)(struct platform_device *pdev, void *priv);
	void (*exit)(struct platform_device *pdev, void *priv);
	struct mac_device_info *(*setup)(void *priv);
	void *bsp_priv;
	struct clk *stmmac_clk;
	struct clk *pclk;
	struct clk *clk_ptp_ref;
	unsigned int clk_ptp_rate;
	struct reset_control *stmmac_rst;
	struct stmmac_axi *axi;
	int has_gmac4;
	bool has_sun8i;
	bool tso_en;
	int mac_port_sel_speed;
	bool en_tx_lpi_clockgating;


};



#endif

