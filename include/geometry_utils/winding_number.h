/**
 * Modified from PhysX's GuWindingNumberT.h, GuWindingNumber.h, and
 * GuWindingNumberT.cpp
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

#ifndef CR_WINDING_NUMBER_H
#define CR_WINDING_NUMBER_H

#include "aabb_tree.h"

namespace crmpm
{
    struct ClusterApproximation
    {
        float Radius;
        float AreaSum;
        Vec3f3 WeightedCentroid;
        Vec3f3 WeightedNormalSum;

        ClusterApproximation() {}

        ClusterApproximation(float radius, float areaSum, const Vec3f3 &weightedCentroid, const Vec3f3 &weightedNormalSum) : Radius(radius), AreaSum(areaSum), WeightedCentroid(weightedCentroid), WeightedNormalSum(weightedNormalSum)
        {
        }
    };

    // Computes parameters to approximately represent a cluster (cluster = bunch of triangles) to be used to compute a winding number approximation
    CRMPM_ENGINE_API void approximateCluster(const Array<int> &triangleSet, unsigned int start, unsigned int end, const unsigned int *triangles, const Vec3f3 *points, const Array<float> &triangleAreas, const Array<Vec3f3> &triangleNormalsTimesTriangleArea, const Array<Vec3f3> &triangleCentroids, ClusterApproximation &cluster);

    struct Section
    {
        int start;
        int end;

        Section() : start(0), end(0) {}
        Section(int s, int e) : start(s), end(e) {}
    };

    // Helper method that recursively traverses the given BVH tree and computes a cluster approximation for every node and links it to the node
    CRMPM_ENGINE_API void precomputeClusterInformation(int nodeId, const AABBTreeNode *tree, const unsigned int *triangles, const unsigned int numTriangles, const Vec3f3 *points, HashMap<unsigned int, ClusterApproximation> &infos, const Array<float> &triangleAreas, const Array<Vec3f3> &triangleNormalsTimesTriangleArea, const Array<Vec3f3> &triangleCentroids);

    // Precomputes a cluster approximation for every node in the BVH tree
    CRMPM_ENGINE_API void precomputeClusterInformation(const AABBTreeNode *tree, const unsigned int *triangles, const unsigned int numTriangles, const Vec3f3 *points, HashMap<unsigned int, ClusterApproximation> &result, int rootNodeIndex = 0);

    CR_FORCE_INLINE float evaluateExact(Vec3f3 a, Vec3f3 b, Vec3f3 c, const Vec3f3 &p)
    {
        const float twoOver4PI = float(0.5 / 3.141592653589793238462643383);

        a -= p;
        b -= p;
        c -= p;

        const float la = a.norm(),
                    lb = b.norm(),
                    lc = c.norm();

        const float y = a.x() * b.y() * c.z() - a.x() * b.z() * c.y() - a.y() * b.x() * c.z() + a.y() * b.z() * c.x() + a.z() * b.x() * c.y() - a.z() * b.y() * c.x();
        const float x = (la * lb * lc + (a.x() * b.x() + a.y() * b.y() + a.z() * b.z()) * lc + (b.x() * c.x() + b.y() * c.y() + b.z() * c.z()) * la + (c.x() * a.x() + c.y() * a.y() + c.z() * a.z()) * lb);
        return twoOver4PI * atan2f(y, x);
    }

    class CRMPM_ENGINE_API WindingNumberTraversalController
    {
    public:
        float mWindingNumber = 0;

    private:
        const unsigned int *mTriangles;
        const Vec3f3 *mPoints;
        const HashMap<unsigned int, ClusterApproximation> &mClusters;
        Vec3f3 mQueryPoint;
        float mDistanceThresholdBeta;

    public:
        WindingNumberTraversalController(
            const unsigned int *triangles,
            const Vec3f3 *points,
            const HashMap<unsigned int, ClusterApproximation> &clusters,
            const Vec3f3 &queryPoint, float distanceThresholdBeta = 2)
            : mTriangles(triangles),
              mPoints(points),
              mClusters(clusters),
              mQueryPoint(queryPoint),
              mDistanceThresholdBeta(distanceThresholdBeta)
        {
        }

        TraversalControl analyze(const AABBTreeNode &node, int nodeIndex);
    };

    CR_FORCE_INLINE float computeWindingNumberApprox(const AABBTreeNode *tree, const Vec3f3 &q, float beta, const HashMap<unsigned int, ClusterApproximation> &clusters, const unsigned int *triangles, const Vec3f3 *points)
    {
        WindingNumberTraversalController c(triangles, points, clusters, q, beta);
        traverseBVH(tree, c);
        return c.mWindingNumber;
    }

    CR_FORCE_INLINE float computeWindingNumberExact(const Vec3f3 &q, const unsigned int *triangles, const unsigned int numTriangles, const Vec3f3 *points)
    {
        float windingNumber = 0.0f;
        for (unsigned int i = 0; i < numTriangles; ++i)
        {
            const unsigned int *tri = &triangles[3 * i];
            windingNumber += evaluateExact(points[tri[0]], points[tri[1]], points[tri[2]], q);
        }
        return windingNumber;
    }

    CR_FORCE_INLINE void buildTree(const unsigned int *triangles, const unsigned int numTriangles, const Vec3f3 *points, Array<AABBTreeNode> &tree, float enlargement = 1e-4f)
    {
        // Computes a bounding box for every triangle in triangles
        Bounds3 *boxes = new Bounds3[numTriangles];
        for (unsigned int i = 0; i < numTriangles; ++i)
        {
            const unsigned int *tri = &triangles[3 * i];
            boxes[i].include(points[tri[0]].cast<float, 4>());
            boxes[i].include(points[tri[1]].cast<float, 4>());
            boxes[i].include(points[tri[2]].cast<float, 4>());
            boxes[i].fattenFast(enlargement);
        }

        buildAABBTree(numTriangles, boxes, tree);

        delete[] boxes;
    }

} // namespace crmpm

#endif // !CR_WINDING_NUMBER_H