/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _KERNEL_SCHED_HUNGARIAN_H_
#define _KERNEL_SCHED_HUNGARIAN_H_

#include <linux/spinlock.h>

#define MAX_VERTICES	64 /* Max number of vertices in each disjoint */

struct hm_graph {
	int		n;
	u64		weight[MAX_VERTICES][MAX_VERTICES];

	/* current state which users access */
	int		n_match;
	int		mate_of_xi[MAX_VERTICES];
	int		mate_of_yi[MAX_VERTICES];
	void		*opaque_x[MAX_VERTICES];
	void		*opaque_y[MAX_VERTICES];

	/* aux */
	u64		lx[MAX_VERTICES];
	u64		ly[MAX_VERTICES];
	int		n_active_x;
	int		n_active_y;

	DECLARE_BITMAP(active_x_mask, MAX_VERTICES);
	DECLARE_BITMAP(active_y_mask, MAX_VERTICES);
	DECLARE_BITMAP(updated_y_mask, MAX_VERTICES);

	raw_spinlock_t	lock;
};

/* TODO: refactor */
typedef void (*hm_updated_ynode_cb_t)(void *arg, int yi);

int hm_mate_of_xi(struct hm_graph *g, int xi);
int hm_mate_of_yi(struct hm_graph *g, int yi);
void *hm_opaque_x(struct hm_graph *g, int xi);
void *hm_opaque_y(struct hm_graph *g, int yi);
void hm_for_each_updated_y(struct hm_graph *g, hm_updated_ynode_cb_t cb, void *arg);
void hm_run(struct hm_graph *g);
void hm_del_x(struct hm_graph *g, int xi);
void hm_del_y(struct hm_graph *g, int yi);
int hm_add_y(struct hm_graph *g, void *opaque);
int hm_add_x(struct hm_graph *g, u64 *weight, int n_weight, void *opaque);
void hm_update_x(struct hm_graph *g, u64 *weight, int n_weight, int xi);
struct hm_graph *hm_init(void);

#endif /* _KERNEL_SCHED_HUNGARIAN_H_ */
