/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_SOURCE_PATHS_H
#define CCAMERA_SOURCE_PATHS_H
/* Owned canonical directory of the active uninstalled image, or NULL if no
 * build tree is identifiable. A resource failure returns a negative errno. */
int cc_build_root(char **root);
#define CC_BUILD_TREE_MAGIC "ccamera-build-tree-v1\n"
#endif
