/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Hungarian method-based IAP implementation:
 */
#include <linux/slab.h>

#include "sched.h"
#include "hungarian.h"

#undef DEBUG
#ifdef DEBUG
#define DBG(...) trace_printk(__VA_ARGS__)
#else
#define DBG(...) do { } while(0)
#endif

static inline void hm_graph_dump(struct hm_graph *g)
{
#ifdef DEBUG
	char row_buf[512] = "";
	int xi, yi, sz;

	DBG("--------\n");
	DBG("  match:\n");
	for (xi = 0; xi < g->n; xi++) {
		/* TODO: this makes debugging easy but obviously not opaque */
		struct task_struct *p = g->opaque_x[xi];
		sz = snprintf(row_buf, sizeof(row_buf)-1, "   %s (%6d) %20llu [ ",
			      test_bit(xi, g->active_x_mask) ? "*" : " ",
			      p ? p->pid : -1, g->lx[xi]);
		for (yi = 0; yi < g->n; yi++) {
			if (sz >= sizeof(row_buf)-1)
				break;
			if (g->mate_of_xi[xi] == yi)
				sz += snprintf(row_buf+sz, sizeof(row_buf)-sz-1,
					       "1(%llu) ", g->weight[xi][yi]);
			else
				sz += snprintf(row_buf+sz, sizeof(row_buf)-sz-1,
					       "0(%llu) ", g->weight[xi][yi]);
		}
		if (sz >= sizeof(row_buf)-1)
			break;
		sz += snprintf(row_buf+sz, sizeof(row_buf)-sz-1, "] %-20llu %s\n",
			       g->ly[xi], test_bit(xi, g->active_y_mask) ? "*" : " ");
		row_buf[sz] = '\0';
		DBG(row_buf);
	}
	DBG("--------\n");
#endif
}

int hm_mate_of_xi(struct hm_graph *g, int xi)
{
	int yi = -1;

	if (xi >= 0 && xi < g->n && test_bit(xi, g->active_x_mask)) {
		yi = g->mate_of_xi[xi];
		if (!test_bit(yi, g->active_y_mask))
			yi = -1;
	}
	return yi;
}

int hm_mate_of_yi(struct hm_graph *g, int yi)
{
	int xi = -1;

	if (yi >= 0 && yi < g->n && test_bit(yi, g->active_y_mask)) {
		xi = g->mate_of_yi[yi];
		if (!test_bit(xi, g->active_x_mask))
			xi = -1;
	}
	return xi;
}

void *hm_opaque_x(struct hm_graph *g, int xi)
{
	if (xi >= 0 && xi < g->n)
		return READ_ONCE(g->opaque_x[xi]);
	return NULL;
}

void *hm_opaque_y(struct hm_graph *g, int yi)
{
	if (yi >= 0 && yi < g->n)
		return READ_ONCE(g->opaque_y[yi]);
	return NULL;
}

void hm_for_each_updated_y(struct hm_graph *g, hm_updated_ynode_cb_t cb, void *arg)
{
	DECLARE_BITMAP(copied_y_mask, MAX_VERTICES);
	unsigned long flags;
	unsigned int yi;

	raw_spin_lock_irqsave(&g->lock, flags);
	bitmap_copy(copied_y_mask, g->updated_y_mask, MAX_VERTICES);
	bitmap_zero(g->updated_y_mask, MAX_VERTICES);
	raw_spin_unlock_irqrestore(&g->lock, flags);

	for_each_set_bit(yi, copied_y_mask, MAX_VERTICES)
		cb(arg, yi);

	return;
}

static inline void hm_revise_labels(struct hm_graph *g, unsigned long *s_mask,
				    unsigned long *t_mask, u64 *slack)
{
	u64 lambda = U64_MAX;
	int i;

	/* calculate lambda using slack */
	for (i = 0; i < g->n; i++)
		if (!test_bit(i, t_mask))
			lambda = min(lambda, slack[i]);

	for (i = 0; i < g->n; i++) {
		/* update X labels */
		if (test_bit(i, s_mask))
			g->lx[i] -= lambda;

		if (test_bit(i, t_mask))
			/* update Y labels */
			g->ly[i] += lambda;
		else
			/* update slack variables */
			slack[i] -= lambda;
	}
}

static inline void hm_reset_labels(struct hm_graph *g)
{
	int i;

	for (i = 0; i < g->n; i++) {
		g->lx[i] = 0;
		g->ly[i] = 0;
	}
}

static inline void hm_update_slack(struct hm_graph *g, int xi, u64 *slack, int *slack_x)
{
	int yi;

	/* update slack */
	for (yi = 0; yi < g->n; yi++) {
		if (g->lx[xi] + g->ly[yi] - g->weight[xi][yi] < slack[yi]) {
			slack[yi] = g->lx[xi] + g->ly[yi] - g->weight[xi][yi];
			slack_x[yi] = xi;
		}
	}
}

static void hm_stage(struct hm_graph *g)
{
	DECLARE_BITMAP(s_mask, MAX_VERTICES);
	DECLARE_BITMAP(t_mask, MAX_VERTICES);
	int prev[MAX_VERTICES] = { -1 };
	int slack_x[MAX_VERTICES];
	u64 slack[MAX_VERTICES];
	int q[MAX_VERTICES];
	int wr = 0, rd = 0;
	int xi, yi, root;
	int mate;

	bitmap_zero(s_mask, MAX_VERTICES);
	bitmap_zero(t_mask, MAX_VERTICES);

	if (g->n_match == g->n)
		return;

	/* find an exposed node */
	for (xi = 0; xi < g->n; xi++) {
		if (g->mate_of_xi[xi] == -1) {
			q[wr++] = root = xi;
			prev[xi] = -2;
			set_bit(xi, s_mask);
			break;
		}
	}

	/* reset slack variables */
	for (yi = 0; yi < g->n; yi++) {
		slack[yi] = g->lx[root] + g->ly[yi] - g->weight[root][yi];
		slack_x[yi] = root;
	}

	/* main cycle */
	while (true) {
		/* build tree with bfs */
		while (rd < wr) {
			xi = q[rd++];
			for (yi = 0; yi < g->n; yi++) {
				if (g->weight[xi][yi] == g->lx[xi] + g->ly[yi] &&
				    !test_bit(yi, t_mask)) {
					/* admissible edge found */
					mate = g->mate_of_yi[yi];
					if (mate == -1)
						/* augmenting path found */
						break;
					set_bit(yi, t_mask);
					q[wr++] = mate;

					set_bit(mate, s_mask);
					prev[mate] = xi;
					hm_update_slack(g, mate, slack, slack_x);
				}
			}
			if (yi < g->n)
				/* augmenting path found */
				break;
		}

		if (yi < g->n)
			/* augmenting path found */
			break;

		/* no augmenting path found, so revise labels in order to add new edges */
		hm_revise_labels(g, s_mask, t_mask, slack);

		/* add new edges to the equality subgraph if possible */
		wr = rd = 0;
		for (yi = 0; yi < g->n; yi++) {
			if (!test_bit(yi, t_mask) && slack[yi] == 0) {
				mate = g->mate_of_yi[yi];
				if (mate == -1) {
					/* this node is exposed - augmenting path found */
					xi = slack_x[yi];
					break;
				}
				set_bit(yi, t_mask);
				if (!test_bit(mate, s_mask)) {
					q[wr++] = mate;

					set_bit(mate, s_mask);
					prev[mate] = slack_x[yi];
					hm_update_slack(g, mate, slack, slack_x);
				}
			}
		}

		if (yi < g->n)
			/* augmenting path found */
			break;
	}

	/* check if an augmenting path found in this stage */
	if (yi < g->n) {
		g->n_match++;

		/* let's augment */
		for (int cx = xi, cy = yi, tmp_yi;
		     cx != -2;
		     cx = prev[cx], cy = tmp_yi) {
			tmp_yi = g->mate_of_xi[cx];
			g->mate_of_yi[cy] = cx;
			g->mate_of_xi[cx] = cy;
			set_bit(cy, g->updated_y_mask);
		}
	}
}

void hm_run(struct hm_graph *g)
{
	unsigned long flags;

	while (g->n_match < g->n) {
		raw_spin_lock_irqsave(&g->lock, flags);
		hm_stage(g);
		hm_graph_dump(g);
		raw_spin_unlock_irqrestore(&g->lock, flags);
	}
}

static void __hm_del_x(struct hm_graph *g, int xi)
{
	int yi;

	/* remove weight */
	for (yi = 0; yi < g->n; yi++)
		g->weight[xi][yi] = 0;

	yi = g->mate_of_xi[xi];

	/*
	 * Until becoming active again, minimum non-zero weight is
	 * kept so that unneeded migration does not occur due to
	 * tie-break.
	 */
	g->weight[xi][yi] = 1;

	/* remove matching */
	g->mate_of_yi[yi] = -1;
	g->mate_of_xi[xi] = -1;

	/* reset label */
	g->lx[xi] = 0;

	/* misc */
	if (test_and_clear_bit(xi, g->active_x_mask))
		--g->n_active_x;
	WRITE_ONCE(g->opaque_x[xi], NULL);
	g->n_match--;

	BUG_ON(g->n_active_x < 0);
	if (g->n_active_x == 0)
		hm_reset_labels(g);
}

void hm_del_x(struct hm_graph *g, int xi)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&g->lock, flags);
	if (xi == -1) {
		raw_spin_unlock_irqrestore(&g->lock, flags);
		return;
	}
	__hm_del_x(g, xi);
	hm_stage(g);
	hm_graph_dump(g);
	raw_spin_unlock_irqrestore(&g->lock, flags);
}

static void __hm_del_y(struct hm_graph *g, int yi)
{
	int xi;

	/* remove weight */
	for (xi = 0; xi < g->n; xi++)
		g->weight[xi][yi] = 0;

	xi = g->mate_of_yi[yi];

	/* remove matching */
	g->mate_of_xi[xi] = -1;
	g->mate_of_yi[yi] = -1;

	/* reset label */
	g->ly[yi] = 0;

	/* misc */
	if (test_and_clear_bit(yi, g->active_y_mask))
		--g->n_active_y;
	WRITE_ONCE(g->opaque_y[yi], NULL);
	g->n_match--;

	BUG_ON(g->n_active_y < 0);
	if (g->n_active_y == 0)
		hm_reset_labels(g);
}

void hm_del_y(struct hm_graph *g, int yi)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&g->lock, flags);
	if (yi == -1) {
		raw_spin_unlock_irqrestore(&g->lock, flags);
		return;
	}
	__hm_del_y(g, yi);
	hm_stage(g);
	hm_graph_dump(g);
	raw_spin_unlock_irqrestore(&g->lock, flags);
}

int hm_add_y(struct hm_graph *g, void *opaque)
{
	unsigned long flags;
	int yi;

	raw_spin_lock_irqsave(&g->lock, flags);
	yi = find_first_zero_bit(g->active_y_mask, MAX_VERTICES);
	if (yi >= MAX_VERTICES) {
		raw_spin_unlock_irqrestore(&g->lock, flags);
		return -ENOSPC;
	}

	if (yi >= g->n)
		g->n += 1; /* TODO: zero re-initialize ?*/
	else
		__hm_del_y(g, yi);

	set_bit(yi, g->active_y_mask);
	g->n_active_y++;

	WRITE_ONCE(g->opaque_y[yi], opaque);
	hm_stage(g);
	hm_graph_dump(g);
	raw_spin_unlock_irqrestore(&g->lock, flags);

	return yi;
}

int hm_add_x(struct hm_graph *g, u64 *weight, int n_weight, void *opaque)
{
	unsigned long flags;
	int xi, yi;

	if (SCHED_WARN_ON(n_weight > MAX_VERTICES))
		n_weight = MAX_VERTICES;

	raw_spin_lock_irqsave(&g->lock, flags);
	xi = find_first_zero_bit(g->active_x_mask, MAX_VERTICES);
	if (xi >= MAX_VERTICES) {
		raw_spin_unlock_irqrestore(&g->lock, flags);
		return -ENOSPC;
	}

	if (xi >= g->n)
		g->n += 1; /* TODO: zero re-initialize ?*/
	else
		__hm_del_x(g, xi);
	set_bit(xi, g->active_x_mask);
	g->n_active_x++;

	for (yi = 0; yi < n_weight; yi++) {
		g->weight[xi][yi] = weight[yi];
		g->lx[xi] = max(g->lx[xi], weight[yi]);
	}

	WRITE_ONCE(g->opaque_x[xi], opaque);
	hm_stage(g);
	hm_graph_dump(g);
	raw_spin_unlock_irqrestore(&g->lock, flags);

	return xi;
}

void hm_update_x(struct hm_graph *g, u64 *weight, int n_weight, int xi)
{
	unsigned long flags;
	void *opaque;
	int yi;

	if (xi < 0)
		return;

	raw_spin_lock_irqsave(&g->lock, flags);
	opaque = g->opaque_x[xi];
	__hm_del_x(g, xi);

	set_bit(xi, g->active_x_mask);
	g->n_active_x++;

	for (yi = 0; yi < n_weight; yi++) {
		g->weight[xi][yi] = weight[yi];
		g->lx[xi] = max(g->lx[xi], weight[yi]);
	}

	WRITE_ONCE(g->opaque_x[xi], opaque);
	hm_stage(g);
	hm_graph_dump(g);
	raw_spin_unlock_irqrestore(&g->lock, flags);
}

struct hm_graph *hm_init(void)
{
	struct hm_graph *g;
	int xi;

	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g)
		return NULL;

	bitmap_zero(g->active_x_mask, MAX_VERTICES);
	bitmap_zero(g->active_y_mask, MAX_VERTICES);
	bitmap_zero(g->updated_y_mask, MAX_VERTICES);

	raw_spin_lock_init(&g->lock);

	for (xi = 0; xi < MAX_VERTICES; xi++) {
		g->mate_of_xi[xi] = -1;
		g->mate_of_yi[xi] = -1;
	}
	return g;
}
