/**
 * Copyright (c) 2010 Anup Patel.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * @file vmm_devdrv.c
 * @author Anup Patel (anup@brainfault.org)
 * @brief Device driver framework source
 */

#include <vmm_error.h>
#include <vmm_stdio.h>
#include <vmm_heap.h>
#include <vmm_devtree.h>
#include <vmm_devdrv.h>
#include <vmm_mutex.h>
#include <libs/stringlib.h>

struct probe_node {
	struct dlist head;
	struct vmm_devtree_node *node;
};

struct vmm_devdrv_ctrl {
	struct vmm_mutex device_lock;
	struct dlist device_list;
	struct vmm_mutex driver_lock;
	struct dlist driver_list;
	struct vmm_mutex class_lock;
	struct dlist class_list;
	struct vmm_mutex pnode_lock;
	struct dlist pnode_list;
};

static struct vmm_devdrv_ctrl ddctrl;

/* Must be called with 'ddctrl.device_lock' held */
static void devdrv_probe(struct vmm_devtree_node *node,
			 struct vmm_driver *drv)
{
	int rc;
	bool found;
	struct dlist *l;
	struct vmm_devtree_node *child;
	struct vmm_device *dinst;
	const struct vmm_devtree_nodeid *match;

	found = FALSE;
	list_for_each(l, &ddctrl.device_list) {
		dinst = list_entry(l, struct vmm_device, head);
		if (dinst->node == node) {
			found = TRUE;
			break;
		}
	}

	if (!found) {
		match = vmm_devtree_match_node(drv->match_table, node);
		if (match) {
			dinst = vmm_malloc(sizeof(struct vmm_device));
			INIT_LIST_HEAD(&dinst->head);
			dinst->node = node;
			dinst->class = NULL;
			dinst->classdev = NULL;
			dinst->drv = drv;
			dinst->priv = NULL;
#if defined(CONFIG_VERBOSE_MODE)
			vmm_printf("Probe device %s\n", node->name);
#endif
			rc = drv->probe(dinst, match);
			if (rc) {
				vmm_printf("%s: %s probe error %d\n", 
					   __func__, node->name, rc);
				vmm_free(dinst);
			} else {
				list_add_tail(&dinst->head, &ddctrl.device_list);
			}
		}
	}

	list_for_each(l, &node->child_list) {
		child = list_entry(l, struct vmm_devtree_node, head);
		devdrv_probe(child, drv);
	}
}

/* Must be called with 'ddctrl.device_lock' held */
static void devdrv_remove(struct vmm_devtree_node *node,
			  struct vmm_driver *drv)
{
	int rc;
	bool found;
	struct dlist *l;
	struct vmm_devtree_node *child;
	struct vmm_device *dinst;

	list_for_each(l, &node->child_list) {
		child = list_entry(l, struct vmm_devtree_node, head);
		devdrv_remove(child, drv);
	}

	found = FALSE;
	list_for_each(l, &ddctrl.device_list) {
		dinst = list_entry(l, struct vmm_device, head);
		if (dinst->node == node) {
			found = TRUE;
			break;
		}
	}

	if (found) {
		if (dinst->drv != drv) {
			return;
		}
#if defined(CONFIG_VERBOSE_MODE)
		vmm_printf("Remove device %s\n", node->name);
#endif
		rc = drv->remove(dinst);
		if (rc) {
			vmm_printf("%s: %s remove error %d\n", 
				   __func__, node->name, rc);
		}
		vmm_free(dinst);
		list_del(&dinst->head);
	}
}

int vmm_devdrv_probe(struct vmm_devtree_node *node)
{
	bool found;
	struct dlist *l;
	struct vmm_driver *drv;
	struct probe_node *pnode;

	if (!node) {
		return VMM_EFAIL;
	}

	vmm_mutex_lock(&ddctrl.pnode_lock);

	found = FALSE;
	list_for_each(l, &ddctrl.pnode_list) {
		pnode = list_entry(l, struct probe_node, head);
		if (pnode->node == node) {
			found = TRUE;
			break;
		}
	}

	if (!found) {
		pnode = vmm_malloc(sizeof(*pnode));
		if (!pnode) {
			vmm_mutex_unlock(&ddctrl.pnode_lock);
			return VMM_ENOMEM;
		}

		INIT_LIST_HEAD(&pnode->head);
		pnode->node = node;

		list_add_tail(&pnode->head, &ddctrl.pnode_list);
	}

	vmm_mutex_unlock(&ddctrl.pnode_lock);

	vmm_mutex_lock(&ddctrl.device_lock);
	vmm_mutex_lock(&ddctrl.driver_lock);

	list_for_each(l, &ddctrl.driver_list) {
		drv = list_entry(l, struct vmm_driver, head);
		devdrv_probe(node, drv);
	}

	vmm_mutex_unlock(&ddctrl.driver_lock);
	vmm_mutex_unlock(&ddctrl.device_lock);

	return VMM_OK;
}

int vmm_devdrv_remove(struct vmm_devtree_node *node)
{
	bool found;
	struct dlist *l;
	struct vmm_driver *drv;
	struct probe_node *pnode;

	if (!node) {
		return VMM_EFAIL;
	}

	vmm_mutex_lock(&ddctrl.pnode_lock);

	found = FALSE;
	list_for_each(l, &ddctrl.pnode_list) {
		pnode = list_entry(l, struct probe_node, head);
		if (pnode->node == node) {
			found = TRUE;
			break;
		}
	}

	if (!found) {
		vmm_mutex_unlock(&ddctrl.pnode_lock);
		return VMM_EFAIL;
	}

	list_del(&pnode->head);

	vmm_mutex_unlock(&ddctrl.pnode_lock);

	vmm_mutex_lock(&ddctrl.device_lock);
	vmm_mutex_lock(&ddctrl.driver_lock);

	list_for_each(l, &ddctrl.driver_list) {
		drv = list_entry(l, struct vmm_driver, head);
		devdrv_remove(node, drv);
	}

	vmm_mutex_unlock(&ddctrl.driver_lock);
	vmm_mutex_unlock(&ddctrl.device_lock);

	vmm_free(pnode);

	return VMM_OK;
}

int vmm_devdrv_register_class(struct vmm_class *cls)
{
	bool found;
	struct dlist *l;
	struct vmm_class *c;

	if (cls == NULL) {
		return VMM_EFAIL;
	}

	c = NULL;
	found = FALSE;

	vmm_mutex_lock(&ddctrl.class_lock);

	list_for_each(l, &ddctrl.class_list) {
		c = list_entry(l, struct vmm_class, head);
		if (strcmp(c->name, cls->name) == 0) {
			found = TRUE;
			break;
		}
	}

	if (found) {
		vmm_mutex_unlock(&ddctrl.class_lock);
		return VMM_EINVALID;
	}

	INIT_LIST_HEAD(&cls->head);
	INIT_SPIN_LOCK(&cls->lock);
	INIT_LIST_HEAD(&cls->classdev_list);

	list_add_tail(&cls->head, &ddctrl.class_list);

	vmm_mutex_unlock(&ddctrl.class_lock);

	return VMM_OK;
}

int vmm_devdrv_unregister_class(struct vmm_class *cls)
{
	bool found;
	struct dlist *l;
	struct vmm_class *c;

	vmm_mutex_lock(&ddctrl.class_lock);

	if (cls == NULL || list_empty(&ddctrl.class_list)) {
		vmm_mutex_unlock(&ddctrl.class_lock);
		return VMM_EFAIL;
	}

	c = NULL;
	found = FALSE;
	list_for_each(l, &ddctrl.class_list) {
		c = list_entry(l, struct vmm_class, head);
		if (strcmp(c->name, cls->name) == 0) {
			found = TRUE;
			break;
		}
	}

	if (!found) {
		vmm_mutex_unlock(&ddctrl.class_lock);
		return VMM_ENOTAVAIL;
	}

	list_del(&c->head);

	vmm_mutex_unlock(&ddctrl.class_lock);

	return VMM_OK;
}

struct vmm_class *vmm_devdrv_find_class(const char *cname)
{
	bool found;
	struct dlist *l;
	struct vmm_class *cls;

	if (!cname) {
		return NULL;
	}

	found = FALSE;
	cls = NULL;

	vmm_mutex_lock(&ddctrl.class_lock);

	list_for_each(l, &ddctrl.class_list) {
		cls = list_entry(l, struct vmm_class, head);
		if (strcmp(cls->name, cname) == 0) {
			found = TRUE;
			break;
		}
	}

	vmm_mutex_unlock(&ddctrl.class_lock);

	if (!found) {
		return NULL;
	}

	return cls;
}

struct vmm_class *vmm_devdrv_class(int index)
{
	bool found;
	struct dlist *l;
	struct vmm_class *retval;

	if (index < 0) {
		return NULL;
	}

	retval = NULL;
	found = FALSE;

	vmm_mutex_lock(&ddctrl.class_lock);

	list_for_each(l, &ddctrl.class_list) {
		retval = list_entry(l, struct vmm_class, head);
		if (!index) {
			found = TRUE;
			break;
		}
		index--;
	}

	vmm_mutex_unlock(&ddctrl.class_lock);

	if (!found) {
		return NULL;
	}

	return retval;
}

u32 vmm_devdrv_class_count(void)
{
	u32 retval;
	struct dlist *l;

	retval = 0;

	vmm_mutex_lock(&ddctrl.class_lock);

	list_for_each(l, &ddctrl.class_list) {
		retval++;
	}

	vmm_mutex_unlock(&ddctrl.class_lock);

	return retval;
}

int vmm_devdrv_register_classdev(const char *cname, struct vmm_classdev *cdev)
{
	bool found;
	irq_flags_t flags;
	struct dlist *l;
	struct vmm_class *c;
	struct vmm_classdev *cd;

	c = vmm_devdrv_find_class(cname);

	if (c == NULL || cdev == NULL) {
		return VMM_EFAIL;
	}

	cd = NULL;
	found = FALSE;

	vmm_spin_lock_irqsave(&c->lock, flags);

	list_for_each(l, &c->classdev_list) {
		cd = list_entry(l, struct vmm_classdev, head);
		if (strcmp(cd->name, cdev->name) == 0) {
			found = TRUE;
			break;
		}
	}

	if (found) {
		vmm_spin_unlock_irqrestore(&c->lock, flags);
		return VMM_EINVALID;
	}

	if (cdev->dev) {
		cdev->dev->class = c;
		cdev->dev->classdev = cdev;
	}
	INIT_LIST_HEAD(&cdev->head);

	list_add_tail(&cdev->head, &c->classdev_list);

	vmm_spin_unlock_irqrestore(&c->lock, flags);

	return VMM_OK;
}

int vmm_devdrv_unregister_classdev(const char *cname, struct vmm_classdev *cdev)
{
	bool found;
	irq_flags_t flags;
	struct dlist *l;
	struct vmm_class *c;
	struct vmm_classdev *cd;

	c = vmm_devdrv_find_class(cname);

	if (c == NULL || cdev == NULL) {
		return VMM_EFAIL;
	}

	vmm_spin_lock_irqsave(&c->lock, flags);

	if (list_empty(&c->classdev_list)) {
		vmm_spin_unlock_irqrestore(&c->lock, flags);
		return VMM_EFAIL;
	}

	cd = NULL;
	found = FALSE;
	list_for_each(l, &c->classdev_list) {
		cd = list_entry(l, struct vmm_classdev, head);
		if (strcmp(cd->name, cdev->name) == 0) {
			found = TRUE;
			break;
		}
	}

	if (!found) {
		vmm_spin_unlock_irqrestore(&c->lock, flags);
		return VMM_ENOTAVAIL;
	}

	if (cd->dev) {
		cd->dev->class = NULL;
		cd->dev->classdev = NULL;
	}

	list_del(&cd->head);

	vmm_spin_unlock_irqrestore(&c->lock, flags);

	return VMM_OK;
}

struct vmm_classdev *vmm_devdrv_find_classdev(const char *cname,
					 const char *cdev_name)
{
	bool found;
	irq_flags_t flags;
	struct dlist *l;
	struct vmm_class *c;
	struct vmm_classdev *cd;

	c = vmm_devdrv_find_class(cname);

	if (c == NULL || cdev_name == NULL) {
		return NULL;
	}

	found = FALSE;
	cd = NULL;

	vmm_spin_lock_irqsave(&c->lock, flags);

	list_for_each(l, &c->classdev_list) {
		cd = list_entry(l, struct vmm_classdev, head);
		if (strcmp(cd->name, cdev_name) == 0) {
			found = TRUE;
			break;
		}
	}

	vmm_spin_unlock_irqrestore(&c->lock, flags);

	if (!found) {
		return NULL;
	}

	return cd;
}

struct vmm_classdev *vmm_devdrv_classdev(const char *cname, int index)
{
	bool found;
	irq_flags_t flags;
	struct dlist *l;
	struct vmm_class *c;
	struct vmm_classdev *retval;

	c = vmm_devdrv_find_class(cname);

	if (c == NULL || index < 0) {
		return NULL;
	}

	retval = NULL;
	found = FALSE;

	vmm_spin_lock_irqsave(&c->lock, flags);

	list_for_each(l, &c->classdev_list) {
		retval = list_entry(l, struct vmm_classdev, head);
		if (!index) {
			found = TRUE;
			break;
		}
		index--;
	}

	vmm_spin_unlock_irqrestore(&c->lock, flags);

	if (!found) {
		return NULL;
	}

	return retval;
}

u32 vmm_devdrv_classdev_count(const char *cname)
{
	u32 retval;
	irq_flags_t flags;
	struct dlist *l;
	struct vmm_class *c;

	c = vmm_devdrv_find_class(cname);

	if (c == NULL) {
		return VMM_EFAIL;
	}

	retval = 0;

	vmm_spin_lock_irqsave(&c->lock, flags);

	list_for_each(l, &c->classdev_list) {
		retval++;
	}

	vmm_spin_unlock_irqrestore(&c->lock, flags);

	return retval;
}

int vmm_devdrv_register_driver(struct vmm_driver *drv)
{
	bool found;
	struct dlist *l;
	struct vmm_driver *d;
	struct probe_node *pnode;

	if (drv == NULL) {
		return VMM_EFAIL;
	}

	d = NULL;
	found = FALSE;

	vmm_mutex_lock(&ddctrl.driver_lock);

	list_for_each(l, &ddctrl.driver_list) {
		d = list_entry(l, struct vmm_driver, head);
		if (strcmp(d->name, drv->name) == 0) {
			found = TRUE;
			break;
		}
	}

	if (found) {
		vmm_mutex_unlock(&ddctrl.driver_lock);
		return VMM_EINVALID;
	}

	INIT_LIST_HEAD(&drv->head);

	list_add_tail(&drv->head, &ddctrl.driver_list);

	vmm_mutex_unlock(&ddctrl.driver_lock);

	vmm_mutex_lock(&ddctrl.device_lock);
	vmm_mutex_lock(&ddctrl.pnode_lock);

	list_for_each(l, &ddctrl.pnode_list) {
		pnode = list_entry(l, struct probe_node, head);
		devdrv_probe(pnode->node, drv);
	}

	vmm_mutex_unlock(&ddctrl.pnode_lock);
	vmm_mutex_unlock(&ddctrl.device_lock);

	return VMM_OK;
}

int vmm_devdrv_unregister_driver(struct vmm_driver *drv)
{
	bool found;
	struct dlist *l;
	struct vmm_driver *d;
	struct probe_node *pnode;

	vmm_mutex_lock(&ddctrl.driver_lock);

	if (drv == NULL || list_empty(&ddctrl.driver_list)) {
		vmm_mutex_unlock(&ddctrl.driver_lock);
		return VMM_EFAIL;
	}

	d = NULL;
	found = FALSE;
	list_for_each(l, &ddctrl.driver_list) {
		d = list_entry(l, struct vmm_driver, head);
		if (strcmp(d->name, drv->name) == 0) {
			found = TRUE;
			break;
		}
	}

	if (!found) {
		vmm_mutex_unlock(&ddctrl.driver_lock);
		return VMM_ENOTAVAIL;
	}

	list_del(&d->head);

	vmm_mutex_unlock(&ddctrl.driver_lock);

	vmm_mutex_lock(&ddctrl.device_lock);
	vmm_mutex_lock(&ddctrl.pnode_lock);

	list_for_each(l, &ddctrl.pnode_list) {
		pnode = list_entry(l, struct probe_node, head);
		devdrv_remove(pnode->node, drv);
	}

	vmm_mutex_unlock(&ddctrl.pnode_lock);
	vmm_mutex_unlock(&ddctrl.device_lock);

	return VMM_OK;
}

struct vmm_driver *vmm_devdrv_find_driver(const char *name)
{
	bool found;
	struct dlist *l;
	struct vmm_driver *drv;

	if (!name) {
		return NULL;
	}

	found = FALSE;
	drv = NULL;

	vmm_mutex_lock(&ddctrl.driver_lock);

	list_for_each(l, &ddctrl.driver_list) {
		drv = list_entry(l, struct vmm_driver, head);
		if (strcmp(drv->name, name) == 0) {
			found = TRUE;
			break;
		}
	}

	vmm_mutex_unlock(&ddctrl.driver_lock);

	if (!found) {
		return NULL;
	}

	return drv;
}

struct vmm_driver *vmm_devdrv_driver(int index)
{
	bool found;
	struct dlist *l;
	struct vmm_driver *retval;

	if (index < 0) {
		return NULL;
	}

	retval = NULL;
	found = FALSE;

	vmm_mutex_lock(&ddctrl.driver_lock);

	list_for_each(l, &ddctrl.driver_list) {
		retval = list_entry(l, struct vmm_driver, head);
		if (!index) {
			found = TRUE;
			break;
		}
		index--;
	}

	vmm_mutex_unlock(&ddctrl.driver_lock);

	if (!found) {
		return NULL;
	}

	return retval;
}

u32 vmm_devdrv_driver_count(void)
{
	u32 retval;
	struct dlist *l;

	retval = 0;

	vmm_mutex_lock(&ddctrl.driver_lock);

	list_for_each(l, &ddctrl.driver_list) {
		retval++;
	}

	vmm_mutex_unlock(&ddctrl.driver_lock);

	return retval;
}

int __init vmm_devdrv_init(void)
{
	memset(&ddctrl, 0, sizeof(ddctrl));

	INIT_MUTEX(&ddctrl.device_lock);
	INIT_LIST_HEAD(&ddctrl.device_list);
	INIT_MUTEX(&ddctrl.driver_lock);
	INIT_LIST_HEAD(&ddctrl.driver_list);
	INIT_MUTEX(&ddctrl.class_lock);
	INIT_LIST_HEAD(&ddctrl.class_list);
	INIT_MUTEX(&ddctrl.pnode_lock);
	INIT_LIST_HEAD(&ddctrl.pnode_list);

	return VMM_OK;
}
