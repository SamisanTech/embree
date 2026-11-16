// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
// Mira / SamisanTech extension: in-place triangle BVH collide + optional export.

#pragma once

#include "rtcore_scene.h"

RTC_NAMESPACE_BEGIN

/* Max children per exported inner node (covers BVH4 and BVH8). */
#define RTC_SCENE_BVH_MAX_BRANCH 8

/*
 * Flattened view of Embree's triangle scene BVH (AABB nodes).
 * Inner: childCount > 0, primCount == 0, children[0..childCount) are node indices.
 * Leaf:  primCount > 0, childCount == 0, primFirst indexes into RTCSceneBVH::primIDs.
 * bounds = {xmin,ymin,zmin,xmax,ymax,zmax}.
 *
 * Prefer rtcCollideTriangleScenes for production collide (zero extra allocation).
 * Export is for debug / tooling only.
 */
struct RTCSceneBVHNode
{
  float bounds[6];
  unsigned children[RTC_SCENE_BVH_MAX_BRANCH];
  unsigned childCount;
  unsigned primFirst;
  unsigned primCount;
};

struct RTCSceneBVH
{
  struct RTCSceneBVHNode* nodes;
  size_t nodeCount;
  unsigned* primIDs;
  size_t primIDCount;
  unsigned root;
  unsigned branchingFactor; /* 4 or 8 */
};

/*
 * In-place dual BVH traversal + triangle–triangle test on committed triangle scenes.
 * Walks Embree NodeRef / AABBNode / Triangle4(v) leaves — no flatten, no extra BVH.
 * offset0 / offset1 may be NULL (treated as zero). Returns true on first hit.
 * Requires triangle BVH4/BVH8 (device config tri_accel=bvh4.triangle4v recommended).
 */
RTC_API bool rtcCollideTriangleScenes(RTCScene scene0, RTCScene scene1,
                                      const float* offset0, const float* offset1);

/*
 * Same walk as rtcCollideTriangleScenes, but each scene may carry a row-major
 * 3x4 (rotation + translation). Null means identity. Triangle vertices stay in
 * mesh-local space; the matrix is applied to bounds and leaf verts.
 * ignore_xyz / ignore_radius (length ignore_count) are world-space spheres that
 * skip scene1 triangles whose closest point lies inside the sphere. Null/0 = none.
 */
RTC_API bool rtcCollideTriangleScenesXform(RTCScene scene0, RTCScene scene1,
                                           const float* xform0_3x4, const float* xform1_3x4,
                                           const float* ignore_xyz, const float* ignore_radius,
                                           unsigned ignore_count);

/*
 * Export the committed scene's triangle BVH into a flat node array (allocates).
 * Prefer rtcCollideTriangleScenes for collide. Free with rtcFreeSceneBVH.
 */
RTC_API bool rtcExportSceneBVH(RTCScene scene, struct RTCSceneBVH* out);

/* Releases memory owned by a successful rtcExportSceneBVH. */
RTC_API void rtcFreeSceneBVH(struct RTCSceneBVH* bvh);

RTC_NAMESPACE_END
