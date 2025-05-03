/**
 * Implementation referred to PhysX's GuAABBTree.h, GuAABBTreeNode.h,
 * GuAABBTreeBounds.h, GuAABBTreeBuildStats.h, GuAABBTree.cpp, GuAABBTreeQuery.h,
 * and GuDistancePointTriangle.cpp
 *
 * https://github.com/NVIDIA-Omniverse/PhysX/tree/dd587fedd79836442a4117164ea8c46685453c34/physx/source/geomutils/src
 *
 * Original code licensed under the BSD 3-Clause License by NVIDIA Corporation.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *  contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 *  OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *  Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
 *  Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
 *  Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
 *
 * Modifications by:
 *  Copyright (c) 2025, Yafei Ou and Mahdi Tavakoli
 *  Licensed under the BSD 3-Clause License.
 */

#include "aabb_tree.h"
#include "constants.h"

namespace crmpm
{
    /**
     * @brief Returns 0 if v.x is largest element of v, 1 if v.y is largest element, 2 if v.z is largest element.
     */
    CR_FORCE_INLINE static unsigned int LargestAxis(const Vec3f3 &v)
    {
        return (fabsf(v.x()) > fabsf(v.y())) ? ((fabsf(v.x()) > fabsf(v.z())) ? 0 : 2) : ((fabsf(v.y()) > fabsf(v.z())) ? 1 : 2);
    }

    static unsigned int reshuffle(unsigned int nb, unsigned int *const CR_RESTRICT prims, const Vec3f3 *CR_RESTRICT centers, float splitValue, unsigned int axis)
    {
        // Shift accessing centers by the largest axis
        const size_t ptrValue = size_t(centers) + axis * sizeof(float);
        const Vec3f3 *CR_RESTRICT centersX = reinterpret_cast<const Vec3f3 *>(ptrValue);

        // Loop through all node-related primitives. Their indices range from mNodePrimitives[0] to mNodePrimitives[mNbPrimitives-1].
        // Those indices map the global list in the tree builder.
        unsigned int nbPos = 0;
        for (unsigned int i = 0; i < nb; i++)
        {
            // Get index in global list
            const unsigned int index = prims[i];

            // Test against the splitting value. The primitive value is tested against the enclosing-box center.
            // [We only need an approximate partition of the enclosing box here.]
            const float primitiveValue = centersX[index].x();

            // Reorganize the list of indices in this order: positive - negative.
            if (primitiveValue > splitValue)
            {
                // Swap entries
                prims[i] = prims[nbPos];
                prims[nbPos] = index;
                // Count primitives assigned to positive space
                nbPos++;
            }
        }
        return nbPos;
    }

    static unsigned int split(const Bounds3 &box, unsigned int nb, unsigned int *const CR_RESTRICT prims, unsigned int axis, const AABBTreeBuildParams &params)
    {
        // Split value = middle of the axis (using only the box)
        float splitValue = box.getCenter(axis);

        return reshuffle(nb, prims, params.mCache, splitValue, axis);
    }

    void AABBTreeBuildNode::subdivide(const AABBTreeBuildParams &params, BuildStats &stats, NodeAllocator &allocator, unsigned int *const indices)
    {
        unsigned int *const CR_RESTRICT primitives = indices + mNodeIndex;
        const unsigned int nbPrims = mNbPrimitives;

        // Compute global box & means for current node. The box is stored in mBV.
        float3 meansV;
        {
            const Bounds3 *CR_RESTRICT boxes = params.mBounds;

            float4 minV = boxes[primitives[0]].minimum.data;
            float4 maxV = boxes[primitives[0]].maximum.data;

            meansV = params.mCache[primitives[0]].data;

            for (unsigned int i = 1; i < nbPrims; i++)
            {
                const unsigned int index = primitives[i];
                const float4 curMinV = boxes[index].minimum.data;
                const float4 curMaxV = boxes[index].maximum.data;
                meansV = meansV + params.mCache[index].data;
                minV = min(minV, curMinV);
                maxV = max(maxV, curMaxV);
            }

            mBV.minimum = minV;
            mBV.maximum = maxV;

            const float coeff = 1.0f / float(nbPrims);
            meansV = meansV * coeff;
        }

        // Check the user-defined limit. Also ensures we stop subdividing if we reach a leaf node.
        if (nbPrims <= params.mLimit)
            return;

        bool validSplit = true;
        unsigned int nbPos;
        {
            // Compute variances
            float3 varsV = make_float3(0, 0, 0);
            for (unsigned int i = 0; i < nbPrims; i++)
            {
                const unsigned int index = primitives[i];
                float3 centerV = params.mCache[index].data;
                centerV = centerV - meansV;
                centerV = centerV * centerV;
                varsV = varsV + centerV;
            }
            const float coeffNb1 = 1.0f / float(nbPrims - 1);
            varsV = varsV * coeffNb1;
            float3 vars;
            vars = varsV;

            // Choose axis with greatest variance
            const unsigned int axis = LargestAxis(Vec3f3(vars.x, vars.y, vars.z));

            // Split along the axis
            nbPos = split(mBV, nbPrims, primitives, axis, params);

            // Check split validity
            if (!nbPos || nbPos == nbPrims)
                validSplit = false;
        }

        // Check the subdivision has been successful
        if (!validSplit)
        {
            // Here, all boxes lie in the same sub-space. Two strategies:
            // - if we are over the split limit, make an arbitrary 50-50 split
            // - else stop subdividing
            if (nbPrims > params.mLimit)
            {
                nbPos = nbPrims >> 1;
            }
            else
                return;
        }

        // Now create children and assign their pointers.
        mPos = allocator.getBiNode();

        stats.mCount += 2;

        // Assign children
        AABBTreeBuildNode *Pos = const_cast<AABBTreeBuildNode *>(mPos);
        AABBTreeBuildNode *Neg = Pos + 1;
        Pos->mNodeIndex = mNodeIndex;
        Pos->mNbPrimitives = nbPos;
        Neg->mNodeIndex = mNodeIndex + nbPos;
        Neg->mNbPrimitives = mNbPrimitives - nbPos;
    }

    // Build AABB tree

    static unsigned int *initAABBTreeBuild(const AABBTreeBuildParams &params, NodeAllocator &nodeAllocator, BuildStats &stats)
    {
        const unsigned int numPrimitives = params.mNbPrimitives;

        if (!numPrimitives)
            return NULL;

        // Init stats
        stats.mCount = 1;

        // Initialize indices. This list will be modified during build.
        unsigned int *indices = new unsigned int[numPrimitives];
        // Identity permutation
        for (unsigned int i = 0; i < numPrimitives; i++)
            indices[i] = i;

        // Allocate a pool of nodes
        nodeAllocator.init(numPrimitives, params.mLimit);

        // Compute box centers only once and cache them
        params.mCache = new Vec3f3[numPrimitives + 1];
        const Bounds3 *CR_RESTRICT boxes = params.mBounds;
        const float half = 0.5f;
        for (unsigned int i = 0; i < numPrimitives; i++)
        {
            const float4 curMinV = boxes[i].minimum.data;
            const float4 curMaxV = boxes[i].maximum.data;
            const float4 centerV = (curMaxV + curMinV) * half;
            params.mCache[i].data = make_float3(centerV.x, centerV.y, centerV.z);
        }

        return indices;
    }

    static void buildHierarchy(AABBTreeBuildNode *root, const AABBTreeBuildParams &params, BuildStats &stats, NodeAllocator &nodeBase, unsigned int *const indices)
    {
        unsigned int nb = 1;
        Array<AABBTreeBuildNode *> stack(CR_AABB_TREE_DEFAULT_BUILD_STACK_SIZE);
        stack.forceSizeUnsafe(CR_AABB_TREE_DEFAULT_BUILD_STACK_SIZE);

        stack[0] = root;

        struct Local
        {
            static CR_FORCE_INLINE unsigned int pushBack(AABBTreeBuildNode *node, Array<AABBTreeBuildNode *> &stack, unsigned int nb)
            {
                stack[nb++] = node;
                if (nb == stack.capacity())
                    stack.forceSizeUnsafe(stack.capacity() * 2);
                return nb;
            }

            static CR_FORCE_INLINE unsigned int processChildren(AABBTreeBuildNode *node, Array<AABBTreeBuildNode *> &stack, unsigned int nb, BuildStats &stats)
            {
                stats.mTotalPrims += node->mNbPrimitives;

                if (!node->isLeaf())
                {
                    AABBTreeBuildNode *Pos = const_cast<AABBTreeBuildNode *>(node->getPos());
                    nb = pushBack(Pos + 1, stack, nb);
                    nb = pushBack(Pos, stack, nb);
                }
                return nb;
            }
        };

        do
        {
            AABBTreeBuildNode *node = stack[--nb];
            node->subdivide(params, stats, nodeBase, indices);
            nb = Local::processChildren(node, stack, nb, stats);

        } while (nb);
    }

    unsigned int *buildAABBTree(const AABBTreeBuildParams &params, NodeAllocator &nodeAllocator, BuildStats &stats)
    {
        unsigned int *indices = initAABBTreeBuild(params, nodeAllocator, stats);
        if (!indices)
            return NULL;

        buildHierarchy(nodeAllocator.mPool, params, stats, nodeAllocator, indices);

        return indices;
    }

    void flattenTree(const NodeAllocator &nodeAllocator, AABBTreeNode *dest, const unsigned int *remap)
    {
        // Gathers all build nodes allocated so far and flatten them to a linear destination array of smaller runtime nodes
        unsigned int offset = 0;
        const unsigned int nbSlabs = nodeAllocator.mSlabs.size();
        for (unsigned int s = 0; s < nbSlabs; s++)
        {
            const NodeAllocator::Slab &currentSlab = nodeAllocator.mSlabs[s];

            AABBTreeBuildNode *pool = currentSlab.mPool;
            for (unsigned int i = 0; i < currentSlab.mNbUsedNodes; i++)
            {
                dest[offset].mBV = pool[i].mBV;
                if (pool[i].isLeaf())
                {
                    unsigned int index = pool[i].mNodeIndex;
                    if (remap)
                        index = remap[index];

                    const unsigned int nbPrims = pool[i].getNbPrimitives();

                    dest[offset].mData = (index << 5) | ((nbPrims & 15) << 1) | 1;
                }
                else
                {
                    unsigned int localNodeIndex = 0xffffffff;
                    unsigned int nodeBase = 0;
                    for (unsigned int j = 0; j < nbSlabs; j++)
                    {
                        if (pool[i].mPos >= nodeAllocator.mSlabs[j].mPool && pool[i].mPos < nodeAllocator.mSlabs[j].mPool + nodeAllocator.mSlabs[j].mNbUsedNodes)
                        {
                            localNodeIndex = pool[i].mPos - nodeAllocator.mSlabs[j].mPool;
                            break;
                        }
                        nodeBase += nodeAllocator.mSlabs[j].mNbUsedNodes;
                    }
                    const unsigned int nodeIndex = nodeBase + localNodeIndex;
                    dest[offset].mData = nodeIndex << 1;
                }
                offset++;
            }
        }
    }

    void buildAABBTree(unsigned int nbBounds, const Bounds3 *CR_RESTRICT bounds, Array<AABBTreeNode> &tree)
    {
        // build the BVH
        BuildStats stats;
        NodeAllocator nodeAllocator;

        unsigned int *indices = buildAABBTree(AABBTreeBuildParams(1, nbBounds, bounds), nodeAllocator, stats);

        // store the computed hierarchy
        tree.resize(stats.mCount);
        flattenTree(nodeAllocator, tree.begin(), indices);

        delete[] indices;
    }

    // AABB tree query

    static float distancePointTriangleSquared(const Vec3f3 &p,
                                              const Vec3f3 &a,
                                              const Vec3f3 &b,
                                              const Vec3f3 &c,
                                              float &u,
                                              float &v,
                                              Vec3f3 &closestP)
    {
        const float zero = 0.0f;
        const float one = 1.0f;

        const Vec3f3 ab = b - a;
        const Vec3f3 ac = c - a;
        const Vec3f3 bc = c - b;
        const Vec3f3 ap = p - a;
        const Vec3f3 bp = p - b;
        const Vec3f3 cp = p - c;

        const float d1 = ab.dot(ap);
        const float d2 = ac.dot(ap);
        const float d3 = ab.dot(bp);
        const float d4 = ac.dot(bp);
        const float d5 = ab.dot(cp);
        const float d6 = ac.dot(cp);

        const float unom = d4 - d3;
        const float udenom = d5 - d6;

        // Check if p in vertex region outside a
        if (d1 <= zero && d2 <= zero)
        {
            u = zero;
            v = zero;
            closestP = a;
            Vec3f3 vv = p - a;
            return vv.dot(vv);
        }

        // Check if p in vertex region outside b
        if (d3 >= zero && d3 >= d4)
        {
            u = one;
            v = zero;
            closestP = b;
            Vec3f3 vv = p - b;
            return vv.dot(vv);
        }

        // Check if p in vertex region outside c
        if (d6 >= zero && d6 >= d5)
        {
            u = zero;
            v = one;
            closestP = c;
            Vec3f3 vv = p - c;
            return vv.dot(vv);
        }

        // Check if p in edge region of AB
        float vc = d1 * d4 - d3 * d2;
        if (vc <= zero && d1 >= zero && d3 <= zero)
        {
            float sScale = d1 / (d1 - d3);
            closestP = a + ab * sScale;
            u = sScale;
            v = zero;
            Vec3f3 vv = p - closestP;
            return vv.dot(vv);
        }

        // Check if p in edge region of BC
        float va = d3 * d6 - d5 * d4;
        if (va <= zero && d4 >= d3 && d5 >= d6)
        {
            float uScale = unom / (unom + udenom);
            closestP = b + bc * uScale;
            u = 1.0f - uScale;
            v = uScale;
            Vec3f3 vv = p - closestP;
            return vv.dot(vv);
        }

        // Check if p in edge region of AC
        float vb = d5 * d2 - d1 * d6;
        if (vb <= zero && d2 >= zero && d6 <= zero)
        {
            float tScale = d2 / (d2 - d6);
            closestP = a + ac * tScale;
            u = zero;
            v = tScale;
            Vec3f3 vv = p - closestP;
            return vv.dot(vv);
        }

        // P must project inside face region. Compute Q using barycentric coordinates
        float denom = 1.0f / (va + vb + vc);
        float t = vb * denom;
        float w = vc * denom;
        closestP = a + ab * t + ac * w;
        u = t;
        v = w;

        Vec3f3 vv = p - closestP;
        return vv.dot(vv);
    }

    float distancePointTriangleSquared2UnitBox(
        const Vec3f3 queryPoint,
        const Vec3f3 triA,
        const Vec3f3 triB,
        const Vec3f3 triC,
        float &u,
        float &v,
        Vec3f3 &closestP)
    {

        const Vec3f3 minimum = min(max(triA, triB), min(triC, queryPoint));
        const Vec3f3 maximum = max(max(triA, triB), max(triC, queryPoint));
        const Vec3f3 size = maximum - minimum;

        float invScaling = fmaxf(1e-12f, fmaxf(size.x(), fmaxf(size.y(), size.z())));
        float scaling = 1 / invScaling;

        const Vec3f3 p = (queryPoint - minimum) * scaling;
        const Vec3f3 a = (triA - minimum) * scaling;
        const Vec3f3 b = (triB - minimum) * scaling;
        const Vec3f3 c = (triC - minimum) * scaling;

        Vec3f3 cp;
        float result = distancePointTriangleSquared(p, a, b, c, u, v, cp);

        closestP = (cp * invScaling) + minimum;

        return result * invScaling * invScaling;
    }

    TraversalControl ClosestDistanceToTrimeshTraversalController::analyze(const AABBTreeNode &node, int)
    {
        if (distancePointBoxSquared(node.mBV, mQueryPoint) >= mClosestDistanceSquared)
            return TraversalControl::eDontGoDeeper;

        if (node.isLeaf())
        {
            const int j = node.getPrimitiveIndex();
            const unsigned int *tri = &mTriangles[3 * j];

            float t1, t2;
            Vec3f3 q = mQueryPoint;
            Vec3f3 a = mPoints[tri[0]];
            Vec3f3 b = mPoints[tri[1]];
            Vec3f3 c = mPoints[tri[2]];
            Vec3f3 closest;
            float d = distancePointTriangleSquared2UnitBox(q, a, b, c, t1, t2, closest);

            if (d < mClosestDistanceSquared)
            {
                mClosestDistanceSquared = d;
                mClosestTriId = j;
                mClosestPoint = closest;
            }
            return TraversalControl::eDontGoDeeper;
        }

        const AABBTreeNode &nodePos = mNodes[node.getPosIndex()];
        const float distSquaredPos = distancePointBoxSquared(nodePos.mBV, mQueryPoint);
        const AABBTreeNode &nodeNeg = mNodes[node.getNegIndex()];
        const float distSquaredNeg = distancePointBoxSquared(nodeNeg.mBV, mQueryPoint);

        if (distSquaredPos < distSquaredNeg)
        {
            if (distSquaredPos < mClosestDistanceSquared)
                return TraversalControl::eGoDeeper;
        }
        else
        {
            if (distSquaredNeg < mClosestDistanceSquared)
                return TraversalControl::eGoDeeperNegFirst;
        }
        return TraversalControl::eDontGoDeeper;
    }

    static float distanceProjectionPointTriangleSquared(
        const Vec3f3 &p,
        const Vec3f3 &a,
        const Vec3f3 &b,
        const Vec3f3 &c,
        const int aIdx,
        const int bIdx,
        const int cIdx,
        HashMap<Pair<int, int>, Array<int>, EdgeHash> &edgeTriangleMap,
        Array<int> &edgeVertices,
        float &u,
        float &v,
        Vec3f3 &closestP,
        bool &projectionInFace,
        bool &projectionOutsideMesh,
        int &side)
    {
        const float zero = 0.0f;
        const float one = 1.0f;

        const Vec3f3 ab = b - a;
        const Vec3f3 ac = c - a;
        const Vec3f3 bc = c - b;
        const Vec3f3 ap = p - a;
        const Vec3f3 bp = p - b;
        const Vec3f3 cp = p - c;

        const float d1 = ab.dot(ap);
        const float d2 = ac.dot(ap);
        const float d3 = ab.dot(bp);
        const float d4 = ac.dot(bp);
        const float d5 = ab.dot(cp);
        const float d6 = ac.dot(cp);

        const float unom = d4 - d3;
        const float udenom = d5 - d6;

        projectionInFace = false;

        Vec3f3 normal = ab.getNormalized().cross(ac.getNormalized());
        normal.normalize(); // Ensure normal is unit length

        float signedDist = ap.dot(normal);

        side = signedDist >= zero ? 1 : -1;

        // Check if p in vertex region outside a
        if (d1 <= zero && d2 <= zero)
        {
            u = zero;
            v = zero;
            closestP = a;
            Vec3f3 vv = p - a;

            if (edgeVertices.find(aIdx) != edgeVertices.end())
            {
                projectionOutsideMesh = true;
            }

            return vv.dot(vv);
        }

        // Check if p in vertex region outside b
        if (d3 >= zero && d3 >= d4)
        {
            u = one;
            v = zero;
            closestP = b;
            Vec3f3 vv = p - b;

            if (edgeVertices.find(bIdx) != edgeVertices.end())
            {
                projectionOutsideMesh = true;
            }

            return vv.dot(vv);
        }

        // Check if p in vertex region outside c
        if (d6 >= zero && d6 >= d5)
        {
            u = zero;
            v = one;
            closestP = c;
            Vec3f3 vv = p - c;

            if (edgeVertices.find(cIdx) != edgeVertices.end())
            {
                projectionOutsideMesh = true;
            }

            return vv.dot(vv);
        }

        // Check if p in edge region of AB
        float vc = d1 * d4 - d3 * d2;
        if (vc <= zero && d1 >= zero && d3 <= zero)
        {
            float sScale = d1 / (d1 - d3);
            closestP = a + ab * sScale;
            u = sScale;
            v = zero;
            Vec3f3 vv = p - closestP;

            Pair<int, int> edge = std::minmax(aIdx, bIdx);
            Array<int> &adjacentTriangles = edgeTriangleMap[edge];
            if (adjacentTriangles.size() < 2)
            {
                projectionOutsideMesh = true;
            }

            return vv.dot(vv);
        }

        // Check if p in edge region of BC
        float va = d3 * d6 - d5 * d4;
        if (va <= zero && d4 >= d3 && d5 >= d6)
        {
            float uScale = unom / (unom + udenom);
            closestP = b + bc * uScale;
            u = 1.0f - uScale;
            v = uScale;
            Vec3f3 vv = p - closestP;

            Pair<int, int> edge = std::minmax(bIdx, cIdx);
            Array<int> &adjacentTriangles = edgeTriangleMap[edge];
            if (adjacentTriangles.size() < 2)
            {
                projectionOutsideMesh = true;
            }

            return vv.dot(vv);
        }

        // Check if p in edge region of AC
        float vb = d5 * d2 - d1 * d6;
        if (vb <= zero && d2 >= zero && d6 <= zero)
        {
            float tScale = d2 / (d2 - d6);
            closestP = a + ac * tScale;
            u = zero;
            v = tScale;
            Vec3f3 vv = p - closestP;

            Pair<int, int> edge = std::minmax(aIdx, cIdx);
            Array<int> &adjacentTriangles = edgeTriangleMap[edge];
            if (adjacentTriangles.size() < 2)
            {
                projectionOutsideMesh = true;
            }

            return vv.dot(vv);
        }

        // P must project inside face region. Compute Q using barycentric coordinates
        projectionInFace = true;
        projectionOutsideMesh = false;
        float denom = 1.0f / (va + vb + vc);
        float t = vb * denom;
        float w = vc * denom;
        closestP = a + ab * t + ac * w;
        u = t;
        v = w;

        Vec3f3 vv = p - closestP;
        return vv.dot(vv);
    }

    static float distanceProjectionPointTriangleSquared2UnitBox(
        const Vec3f3 queryPoint,
        const Vec3f3 triA,
        const Vec3f3 triB,
        const Vec3f3 triC,
        const int triAIdx,
        const int triBIdx,
        const int triCIdx,
        HashMap<Pair<int, int>, Array<int>, EdgeHash> &edgeTriangleMap,
        Array<int> &edgeVertices,
        float &u,
        float &v,
        Vec3f3 &closestP,
        bool &projectionInFace,
        bool &projectionOutsideMesh,
        int &side)
    {

        const Vec3f3 minimum = min(max(triA, triB), min(triC, queryPoint));
        const Vec3f3 maximum = max(max(triA, triB), max(triC, queryPoint));
        const Vec3f3 size = maximum - minimum;

        float invScaling = fmaxf(1e-12f, fmaxf(size.x(), fmaxf(size.y(), size.z())));
        float scaling = 1 / invScaling;

        const Vec3f3 p = (queryPoint - minimum) * scaling;
        const Vec3f3 a = (triA - minimum) * scaling;
        const Vec3f3 b = (triB - minimum) * scaling;
        const Vec3f3 c = (triC - minimum) * scaling;

        Vec3f3 cp;
        float result = distanceProjectionPointTriangleSquared(p, a, b, c, triAIdx, triBIdx, triCIdx, edgeTriangleMap, edgeVertices, u, v, cp, projectionInFace, projectionOutsideMesh, side);

        closestP = (cp * invScaling) + minimum;

        return result * invScaling * invScaling;
    }

    TraversalControl ClosestDistanceProjectionToTrimeshTraversalController::analyze(const AABBTreeNode &node, int)
    {
        if (distancePointBoxSquared(node.mBV, mQueryPoint) >= mClosestDistanceSquared)
            return TraversalControl::eDontGoDeeper;

        if (node.isLeaf())
        {
            const int j = node.getPrimitiveIndex();
            const unsigned int *tri = &mTriangles[3 * j];

            float t1, t2;
            Vec3f3 q = mQueryPoint;
            Vec3f3 a = mPoints[tri[0]];
            Vec3f3 b = mPoints[tri[1]];
            Vec3f3 c = mPoints[tri[2]];
            Vec3f3 closest;
            bool projectionInFace = false;
            bool projectionOutsideMesh = false;
            int side = 0;
            float d = distanceProjectionPointTriangleSquared2UnitBox(q, a, b, c, tri[0], tri[1], tri[2], *mEdgeTriangleMap, *mEdgeVertices, t1, t2, closest, projectionInFace, projectionOutsideMesh, side);

            if (d <= mClosestDistanceSquared)
            {
                mClosestDistanceSquared = d;
                mClosestTriId = j;
                mClosestPoint = closest;

                if (projectionOutsideMesh)
                {
                    mProjectionOnSurface = false;
                }

                mSide = side;
            }
            return TraversalControl::eDontGoDeeper;
        }

        const AABBTreeNode &nodePos = mNodes[node.getPosIndex()];
        const float distSquaredPos = distancePointBoxSquared(nodePos.mBV, mQueryPoint);
        const AABBTreeNode &nodeNeg = mNodes[node.getNegIndex()];
        const float distSquaredNeg = distancePointBoxSquared(nodeNeg.mBV, mQueryPoint);

        if (distSquaredPos < distSquaredNeg)
        {
            if (distSquaredPos < mClosestDistanceSquared)
                return TraversalControl::eGoDeeper;
        }
        else
        {
            if (distSquaredNeg < mClosestDistanceSquared)
                return TraversalControl::eGoDeeperNegFirst;
        }
        return TraversalControl::eDontGoDeeper;
    }

} // namespace crmpm
