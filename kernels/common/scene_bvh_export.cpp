// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
// Mira / SamisanTech: in-place triangle BVH collide + optional flatten export.

#define RTC_EXPORT_API

#include "default.h"
#include "scene.h"
#include "accel.h"
#include "acceln.h"

#include "../bvh/bvh.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/triangle_triangle_intersector.h"

#include "../../include/embree4/rtcore_scene_bvh.h"

#include <cstring>
#include <string>
#include <vector>

namespace embree {

namespace {

void set_bounds6(float out[6], const BBox3fa& b)
{
  out[0] = b.lower.x; out[1] = b.lower.y; out[2] = b.lower.z;
  out[3] = b.upper.x; out[4] = b.upper.y; out[5] = b.upper.z;
}

template<typename Prim>
void append_leaf_prims(char* ptr, size_t numBlocks, std::vector<unsigned>& primIDs)
{
  auto* prims = (Prim*)ptr;
  for (size_t i = 0; i < numBlocks; ++i) {
    for (size_t j = 0; j < Prim::max_size(); ++j) {
      if (prims[i].valid(j))
        primIDs.push_back(prims[i].primID(j));
    }
  }
}

struct ExportState {
  std::vector<RTCSceneBVHNode> nodes;
  std::vector<unsigned> primIDs;
  const PrimitiveType* primTy = nullptr;
};

template<int N>
unsigned export_ref(typename BVHN<N>::NodeRef ref, const BBox3fa& knownBounds, ExportState& st)
{
  using NodeRef = typename BVHN<N>::NodeRef;
  using AABBNode = typename BVHN<N>::AABBNode;

  if (ref == NodeRef::emptyNode)
    return (unsigned)-1;

  if (ref.isLeaf()) {
    size_t num = 0;
    char* leaf = ref.leaf(num);
    RTCSceneBVHNode node;
    std::memset(&node, 0, sizeof(node));
    for (unsigned i = 0; i < RTC_SCENE_BVH_MAX_BRANCH; ++i)
      node.children[i] = (unsigned)-1;
    node.childCount = 0;
    node.primFirst = (unsigned)st.primIDs.size();

    const char* name = st.primTy ? st.primTy->name() : "";
    if (std::string(name) == "triangle4v")
      append_leaf_prims<Triangle4v>(leaf, num, st.primIDs);
    else if (std::string(name) == "triangle4")
      append_leaf_prims<Triangle4>(leaf, num, st.primIDs);
    else {
      // Best-effort: treat as Triangle4v layout (common static triangle accel).
      append_leaf_prims<Triangle4v>(leaf, num, st.primIDs);
    }

    node.primCount = (unsigned)st.primIDs.size() - node.primFirst;
    set_bounds6(node.bounds, knownBounds);
    const unsigned idx = (unsigned)st.nodes.size();
    st.nodes.push_back(node);
    return idx;
  }

  if (!ref.isAABBNode()) {
    // Unsupported node type (MB / OBB / quantized) for this export path.
    return (unsigned)-1;
  }

  AABBNode* anode = ref.getAABBNode();
  unsigned childIdx[N];
  unsigned childCount = 0;
  BBox3fa unionB(empty);

  for (size_t i = 0; i < N; ++i) {
    NodeRef c = anode->child(i);
    if (c == NodeRef::emptyNode)
      continue;
    const BBox3fa cb = anode->bounds(i);
    const unsigned ci = export_ref<N>(c, cb, st);
    if (ci == (unsigned)-1)
      continue;
    childIdx[childCount++] = ci;
    unionB.extend(cb);
  }

  RTCSceneBVHNode node;
  std::memset(&node, 0, sizeof(node));
  for (unsigned i = 0; i < RTC_SCENE_BVH_MAX_BRANCH; ++i)
    node.children[i] = (unsigned)-1;
  node.childCount = childCount;
  node.primFirst = 0;
  node.primCount = 0;
  for (unsigned i = 0; i < childCount; ++i)
    node.children[i] = childIdx[i];
  set_bounds6(node.bounds, childCount ? unionB : knownBounds);

  const unsigned idx = (unsigned)st.nodes.size();
  st.nodes.push_back(node);
  return idx;
}

BVH4* find_bvh4(AccelData* accel)
{
  if (!accel)
    return nullptr;
  if (accel->type == AccelData::TY_BVH4)
    return (BVH4*)accel;
  if (accel->type == AccelData::TY_ACCELN) {
    AccelN* accelN = (AccelN*)accel;
    BVH4* fallback = nullptr;
    for (size_t i = 0; i < accelN->accels.size(); ++i) {
      Accel* a = accelN->accels[i];
      if (!a || !a->intersectors.ptr)
        continue;
      if (a->intersectors.ptr->type != AccelData::TY_BVH4)
        continue;
      BVH4* bvh = (BVH4*)a->intersectors.ptr;
      if (!bvh->primTy)
        continue;
      const std::string name = bvh->primTy->name();
      if (name == "triangle4v" || name == "triangle4")
        return bvh;
      if (!fallback)
        fallback = bvh;
    }
    return fallback;
  }
  return nullptr;
}

BVH8* find_bvh8(AccelData* accel)
{
  if (!accel)
    return nullptr;
  if (accel->type == AccelData::TY_BVH8)
    return (BVH8*)accel;
  if (accel->type == AccelData::TY_ACCELN) {
    AccelN* accelN = (AccelN*)accel;
    BVH8* fallback = nullptr;
    for (size_t i = 0; i < accelN->accels.size(); ++i) {
      Accel* a = accelN->accels[i];
      if (!a || !a->intersectors.ptr)
        continue;
      if (a->intersectors.ptr->type != AccelData::TY_BVH8)
        continue;
      BVH8* bvh = (BVH8*)a->intersectors.ptr;
      if (!bvh->primTy)
        continue;
      const std::string name = bvh->primTy->name();
      if (name == "triangle4v" || name == "triangle4")
        return bvh;
      if (!fallback)
        fallback = bvh;
    }
    return fallback;
  }
  return nullptr;
}

bool export_bvh(Accel* sceneAccel, RTCSceneBVH* out)
{
  AccelData* ptr = sceneAccel->intersectors.ptr;
  ExportState st;

  if (BVH4* bvh4 = find_bvh4(ptr)) {
    st.primTy = bvh4->primTy;
    const BBox3fa rootBounds = bvh4->getBounds();
    const unsigned root = export_ref<4>(bvh4->root, rootBounds, st);
    if (root == (unsigned)-1 || st.nodes.empty())
      return false;

    out->nodes = (RTCSceneBVHNode*)malloc(sizeof(RTCSceneBVHNode) * st.nodes.size());
    out->primIDs = st.primIDs.empty() ? nullptr : (unsigned*)malloc(sizeof(unsigned) * st.primIDs.size());
    if (!out->nodes || (!st.primIDs.empty() && !out->primIDs)) {
      free(out->nodes);
      free(out->primIDs);
      out->nodes = nullptr;
      out->primIDs = nullptr;
      return false;
    }
    std::memcpy(out->nodes, st.nodes.data(), sizeof(RTCSceneBVHNode) * st.nodes.size());
    if (!st.primIDs.empty())
      std::memcpy(out->primIDs, st.primIDs.data(), sizeof(unsigned) * st.primIDs.size());
    out->nodeCount = st.nodes.size();
    out->primIDCount = st.primIDs.size();
    out->root = root;
    out->branchingFactor = 4;
    return true;
  }

  if (BVH8* bvh8 = find_bvh8(ptr)) {
    st.primTy = bvh8->primTy;
    const BBox3fa rootBounds = bvh8->getBounds();
    const unsigned root = export_ref<8>(bvh8->root, rootBounds, st);
    if (root == (unsigned)-1 || st.nodes.empty())
      return false;

    out->nodes = (RTCSceneBVHNode*)malloc(sizeof(RTCSceneBVHNode) * st.nodes.size());
    out->primIDs = st.primIDs.empty() ? nullptr : (unsigned*)malloc(sizeof(unsigned) * st.primIDs.size());
    if (!out->nodes || (!st.primIDs.empty() && !out->primIDs)) {
      free(out->nodes);
      free(out->primIDs);
      out->nodes = nullptr;
      out->primIDs = nullptr;
      return false;
    }
    std::memcpy(out->nodes, st.nodes.data(), sizeof(RTCSceneBVHNode) * st.nodes.size());
    if (!st.primIDs.empty())
      std::memcpy(out->primIDs, st.primIDs.data(), sizeof(unsigned) * st.primIDs.size());
    out->nodeCount = st.nodes.size();
    out->primIDCount = st.primIDs.size();
    out->root = root;
    out->branchingFactor = 8;
    return true;
  }

  return false;
}

__forceinline BBox3fa translate_box(const BBox3fa& b, const Vec3fa& o)
{
  return BBox3fa(b.lower + o, b.upper + o);
}

__forceinline bool boxes_overlap(const BBox3fa& a, const BBox3fa& b)
{
  return !(a.upper.x < b.lower.x || a.lower.x > b.upper.x ||
           a.upper.y < b.lower.y || a.lower.y > b.upper.y ||
           a.upper.z < b.lower.z || a.lower.z > b.upper.z);
}

__forceinline Vec3fa tri4v_vert(const Vec3vf<4>& v, size_t i)
{
  return Vec3fa(v.x[i], v.y[i], v.z[i]);
}

template<typename Prim>
bool leaf_leaf_prim_pairs(char* leaf0, size_t n0, const Vec3fa& o0,
                          char* leaf1, size_t n1, const Vec3fa& o1)
{
  auto* p0 = (Prim*)leaf0;
  auto* p1 = (Prim*)leaf1;
  for (size_t i = 0; i < n0; ++i) {
    for (size_t a = 0; a < Prim::max_size(); ++a) {
      if (!p0[i].valid(a))
        continue;
      const Vec3fa a0 = tri4v_vert(p0[i].v0, a) + o0;
      const Vec3fa a1 = tri4v_vert(p0[i].v1, a) + o0;
      const Vec3fa a2 = tri4v_vert(p0[i].v2, a) + o0;
      for (size_t j = 0; j < n1; ++j) {
        for (size_t b = 0; b < Prim::max_size(); ++b) {
          if (!p1[j].valid(b))
            continue;
          const Vec3fa b0 = tri4v_vert(p1[j].v0, b) + o1;
          const Vec3fa b1 = tri4v_vert(p1[j].v1, b) + o1;
          const Vec3fa b2 = tri4v_vert(p1[j].v2, b) + o1;
          if (isa::TriangleTriangleIntersector::intersect_triangle_triangle(a0, a1, a2, b0, b1, b2))
            return true;
        }
      }
    }
  }
  return false;
}

/* Triangle4 stores v0/e1/e2 — recover vertices. */
bool leaf_leaf_triangle4(char* leaf0, size_t n0, const Vec3fa& o0,
                         char* leaf1, size_t n1, const Vec3fa& o1)
{
  auto* p0 = (Triangle4*)leaf0;
  auto* p1 = (Triangle4*)leaf1;
  for (size_t i = 0; i < n0; ++i) {
    for (size_t a = 0; a < Triangle4::max_size(); ++a) {
      if (!p0[i].valid(a))
        continue;
      const Vec3fa v0 = tri4v_vert(p0[i].v0, a);
      const Vec3fa e1 = tri4v_vert(p0[i].e1, a);
      const Vec3fa e2 = tri4v_vert(p0[i].e2, a);
      const Vec3fa a0 = v0 + o0;
      const Vec3fa a1 = v0 - e1 + o0;
      const Vec3fa a2 = v0 + e2 + o0;
      for (size_t j = 0; j < n1; ++j) {
        for (size_t b = 0; b < Triangle4::max_size(); ++b) {
          if (!p1[j].valid(b))
            continue;
          const Vec3fa u0 = tri4v_vert(p1[j].v0, b);
          const Vec3fa f1 = tri4v_vert(p1[j].e1, b);
          const Vec3fa f2 = tri4v_vert(p1[j].e2, b);
          const Vec3fa b0 = u0 + o1;
          const Vec3fa b1 = u0 - f1 + o1;
          const Vec3fa b2 = u0 + f2 + o1;
          if (isa::TriangleTriangleIntersector::intersect_triangle_triangle(a0, a1, a2, b0, b1, b2))
            return true;
        }
      }
    }
  }
  return false;
}

struct CollidePrimTy {
  enum Kind { Tri4v, Tri4, Unknown } kind = Unknown;
  static Kind from_name(const char* name)
  {
    if (!name)
      return Unknown;
    if (std::string(name) == "triangle4v")
      return Tri4v;
    if (std::string(name) == "triangle4")
      return Tri4;
    return Unknown;
  }
};

template<int N>
bool process_leaves(typename BVHN<N>::NodeRef leaf0, CollidePrimTy::Kind ty0,
                    typename BVHN<N>::NodeRef leaf1, CollidePrimTy::Kind ty1,
                    const Vec3fa& o0, const Vec3fa& o1)
{
  size_t n0 = 0, n1 = 0;
  char* p0 = leaf0.leaf(n0);
  char* p1 = leaf1.leaf(n1);
  if (ty0 == CollidePrimTy::Tri4v && ty1 == CollidePrimTy::Tri4v)
    return leaf_leaf_prim_pairs<Triangle4v>(p0, n0, o0, p1, n1, o1);
  if (ty0 == CollidePrimTy::Tri4 && ty1 == CollidePrimTy::Tri4)
    return leaf_leaf_triangle4(p0, n0, o0, p1, n1, o1);
  /* Mixed / unknown: treat as triangle4v (device forced to that layout). */
  return leaf_leaf_prim_pairs<Triangle4v>(p0, n0, o0, p1, n1, o1);
}

template<int N>
bool collide_recurse(typename BVHN<N>::NodeRef ref0, const BBox3fa& bounds0,
                     typename BVHN<N>::NodeRef ref1, const BBox3fa& bounds1,
                     const Vec3fa& o0, const Vec3fa& o1,
                     CollidePrimTy::Kind ty0, CollidePrimTy::Kind ty1)
{
  using NodeRef = typename BVHN<N>::NodeRef;
  using AABBNode = typename BVHN<N>::AABBNode;

  if (ref0 == NodeRef::emptyNode || ref1 == NodeRef::emptyNode)
    return false;
  if (!boxes_overlap(translate_box(bounds0, o0), translate_box(bounds1, o1)))
    return false;

  const bool leaf0 = ref0.isLeaf();
  const bool leaf1 = ref1.isLeaf();

  if (leaf0 && leaf1)
    return process_leaves<N>(ref0, ty0, ref1, ty1, o0, o1);

  if (leaf0) {
    if (!ref1.isAABBNode())
      return false;
    AABBNode* node1 = ref1.getAABBNode();
    const BBox3fa b0t = translate_box(bounds0, o0);
    for (size_t i = 0; i < N; ++i) {
      NodeRef c = node1->child(i);
      if (c == NodeRef::emptyNode)
        continue;
      const BBox3fa cb = node1->bounds(i);
      if (!boxes_overlap(b0t, translate_box(cb, o1)))
        continue;
      if (collide_recurse<N>(ref0, bounds0, c, cb, o0, o1, ty0, ty1))
        return true;
    }
    return false;
  }

  if (leaf1) {
    if (!ref0.isAABBNode())
      return false;
    AABBNode* node0 = ref0.getAABBNode();
    const BBox3fa b1t = translate_box(bounds1, o1);
    for (size_t i = 0; i < N; ++i) {
      NodeRef c = node0->child(i);
      if (c == NodeRef::emptyNode)
        continue;
      const BBox3fa cb = node0->bounds(i);
      if (!boxes_overlap(translate_box(cb, o0), b1t))
        continue;
      if (collide_recurse<N>(c, cb, ref1, bounds1, o0, o1, ty0, ty1))
        return true;
    }
    return false;
  }

  if (!ref0.isAABBNode() || !ref1.isAABBNode())
    return false;

  /* Split the larger volume first (same heuristic as Embree collider). */
  if (area(bounds0) > area(bounds1)) {
    AABBNode* node0 = ref0.getAABBNode();
    const BBox3fa b1t = translate_box(bounds1, o1);
    for (size_t i = 0; i < N; ++i) {
      NodeRef c = node0->child(i);
      if (c == NodeRef::emptyNode)
        continue;
      const BBox3fa cb = node0->bounds(i);
      if (!boxes_overlap(translate_box(cb, o0), b1t))
        continue;
      if (collide_recurse<N>(c, cb, ref1, bounds1, o0, o1, ty0, ty1))
        return true;
    }
  } else {
    AABBNode* node1 = ref1.getAABBNode();
    const BBox3fa b0t = translate_box(bounds0, o0);
    for (size_t i = 0; i < N; ++i) {
      NodeRef c = node1->child(i);
      if (c == NodeRef::emptyNode)
        continue;
      const BBox3fa cb = node1->bounds(i);
      if (!boxes_overlap(b0t, translate_box(cb, o1)))
        continue;
      if (collide_recurse<N>(ref0, bounds0, c, cb, o0, o1, ty0, ty1))
        return true;
    }
  }
  return false;
}

template<int N>
bool collide_bvh(BVHN<N>* bvh0, BVHN<N>* bvh1, const Vec3fa& o0, const Vec3fa& o1)
{
  if (!bvh0 || !bvh1)
    return false;
  const CollidePrimTy::Kind ty0 = CollidePrimTy::from_name(bvh0->primTy ? bvh0->primTy->name() : "");
  const CollidePrimTy::Kind ty1 = CollidePrimTy::from_name(bvh1->primTy ? bvh1->primTy->name() : "");
  return collide_recurse<N>(bvh0->root, bvh0->getBounds(), bvh1->root, bvh1->getBounds(),
                            o0, o1, ty0, ty1);
}

bool collide_triangle_scenes(Accel* a0, Accel* a1, const Vec3fa& o0, const Vec3fa& o1)
{
  AccelData* p0 = a0->intersectors.ptr;
  AccelData* p1 = a1->intersectors.ptr;

  BVH4* b40 = find_bvh4(p0);
  BVH4* b41 = find_bvh4(p1);
  if (b40 && b41)
    return collide_bvh<4>(b40, b41, o0, o1);

  BVH8* b80 = find_bvh8(p0);
  BVH8* b81 = find_bvh8(p1);
  if (b80 && b81)
    return collide_bvh<8>(b80, b81, o0, o1);

  return false;
}

} // namespace

} // namespace embree

using namespace embree;

RTC_NAMESPACE_BEGIN

RTC_API bool rtcCollideTriangleScenes(RTCScene hscene0, RTCScene hscene1,
                                      const float* offset0, const float* offset1)
{
  Scene* scene0 = (Scene*)hscene0;
  Scene* scene1 = (Scene*)hscene1;
  RTC_CATCH_BEGIN;
  RTC_TRACE(rtcCollideTriangleScenes);
  RTC_VERIFY_HANDLE(hscene0);
  RTC_VERIFY_HANDLE(hscene1);
  const Vec3fa o0 = offset0 ? Vec3fa(offset0[0], offset0[1], offset0[2]) : Vec3fa(0.f);
  const Vec3fa o1 = offset1 ? Vec3fa(offset1[0], offset1[1], offset1[2]) : Vec3fa(0.f);
  /* Identical scene + identical offset ⇒ every triangle overlaps itself. */
  if (scene0 == scene1 && o0.x == o1.x && o0.y == o1.y && o0.z == o1.z)
    return true;
  return collide_triangle_scenes((Accel*)scene0, (Accel*)scene1, o0, o1);
  RTC_CATCH_END2(scene0);
  return false;
}

RTC_API bool rtcExportSceneBVH(RTCScene hscene, struct RTCSceneBVH* out)
{
  Scene* scene = (Scene*)hscene;
  RTC_CATCH_BEGIN;
  RTC_TRACE(rtcExportSceneBVH);
  RTC_VERIFY_HANDLE(hscene);
  if (!out)
    throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "out is null");
  std::memset(out, 0, sizeof(*out));
  if (!export_bvh((Accel*)scene, out))
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,
                   "scene has no exportable triangle BVH4/BVH8 (use tri_accel=bvh4.triangle4v)");
  return true;
  RTC_CATCH_END2(scene);
  return false;
}

RTC_API void rtcFreeSceneBVH(struct RTCSceneBVH* bvh)
{
  if (!bvh)
    return;
  free(bvh->nodes);
  free(bvh->primIDs);
  std::memset(bvh, 0, sizeof(*bvh));
}

RTC_NAMESPACE_END
