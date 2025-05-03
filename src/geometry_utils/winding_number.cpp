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

#include "winding_number.h"

namespace crmpm
{
    void approximateCluster(const Array<int> &triangleSet, unsigned int start, unsigned int end, const unsigned int *triangles, const Vec3f3 *points, const Array<float> &triangleAreas, const Array<Vec3f3> &triangleNormalsTimesTriangleArea, const Array<Vec3f3> &triangleCentroids, ClusterApproximation &cluster)
    {
        Vec3f3 weightedCentroid(0., 0., 0.);
        float areaSum = 0;
        Vec3f3 weightedNormalSum(0., 0., 0.);

        for (unsigned int i = start; i < end; ++i)
        {
            int triId = triangleSet[i];
            areaSum += triangleAreas[triId];
            weightedCentroid += triangleCentroids[triId] * triangleAreas[triId];
            weightedNormalSum += triangleNormalsTimesTriangleArea[triId];
        }
        weightedCentroid = weightedCentroid / areaSum;

        float radiusSquared = 0;
        for (unsigned int i = start; i < end; ++i)
        {
            int triId = triangleSet[i];
            const unsigned int *tri = &triangles[3 * triId];
            float d2 = (weightedCentroid - points[tri[0]]).squaredNorm();
            if (d2 > radiusSquared)
                radiusSquared = d2;
            d2 = (weightedCentroid - points[tri[1]]).squaredNorm();
            if (d2 > radiusSquared)
                radiusSquared = d2;
            d2 = (weightedCentroid - points[tri[2]]).squaredNorm();
            if (d2 > radiusSquared)
                radiusSquared = d2;
        }
        cluster = ClusterApproximation(sqrtf(radiusSquared), areaSum, weightedCentroid, weightedNormalSum);
    }

    void precomputeClusterInformation(int nodeId, const AABBTreeNode *tree, const unsigned int *triangles, const unsigned int numTriangles, const Vec3f3 *points, HashMap<unsigned int, ClusterApproximation> &infos, const Array<float> &triangleAreas, const Array<Vec3f3> &triangleNormalsTimesTriangleArea, const Array<Vec3f3> &triangleCentroids)
    {
        Array<int> stack;
        stack.pushBack(nodeId);
        Array<Section> returnStack;

        Array<int> triIndices;
        triIndices.reserve(numTriangles);
        infos.reserve((unsigned int)(1.2f * numTriangles));

        while (stack.size() > 0)
        {
            nodeId = stack.popBack();

            if (nodeId >= 0)
            {
                const AABBTreeNode &node = tree[nodeId];
                if (node.isLeaf())
                {
                    triIndices.pushBack(node.getPrimitiveIndex());
                    returnStack.pushBack(Section(triIndices.size() - 1, triIndices.size()));
                    continue;
                }

                stack.pushBack(-nodeId - 1); // Marker for return index
                stack.pushBack(node.getPosIndex());
                stack.pushBack(node.getPosIndex() + 1);
            }
            else
            {
                Section trianglesA = returnStack.popBack();
                Section trianglesB = returnStack.popBack();
                Section sum(trianglesB.start, trianglesA.end);

                nodeId = -nodeId - 1;
                ClusterApproximation c;
                approximateCluster(triIndices, sum.start, sum.end, triangles, points, triangleAreas, triangleNormalsTimesTriangleArea, triangleCentroids, c);
                infos.insert({(unsigned int)(nodeId), c});

                returnStack.pushBack(sum);
            }
        }
    }

    void precomputeClusterInformation(const AABBTreeNode *tree, const unsigned int *triangles, const unsigned int numTriangles, const Vec3f3 *points, HashMap<unsigned int, ClusterApproximation> &result, int rootNodeIndex)
    {
        Array<float> triangleAreas;
        triangleAreas.resize(numTriangles);
        Array<Vec3f3> triangleNormalsTimesTriangleArea;
        triangleNormalsTimesTriangleArea.resize(numTriangles);
        Array<Vec3f3> triangleCentroids;
        triangleCentroids.resize(numTriangles);

        for (unsigned int i = 0; i < numTriangles; ++i)
        {
            const unsigned int *tri = &triangles[3 * i];
            const Vec3f3 &a = points[tri[0]];
            const Vec3f3 &b = points[tri[1]];
            const Vec3f3 &c = points[tri[2]];
            triangleNormalsTimesTriangleArea[i] = (b - a).cross(c - a) * float(0.5);
            triangleAreas[i] = triangleNormalsTimesTriangleArea[i].norm();
            triangleCentroids[i] = (a + b + c) * float(1.0 / 3.0);
        }

        result.clear();
        precomputeClusterInformation(rootNodeIndex, tree, triangles, numTriangles, points, result, triangleAreas, triangleNormalsTimesTriangleArea, triangleCentroids);
    }

    static float firstOrderClusterApproximation(const Vec3f3 &weightedCentroid, const Vec3f3 &weightedNormalSum,
                                                const Vec3f3 &evaluationPoint)
    {
        const Vec3f3 dir = weightedCentroid - evaluationPoint;
        const float l = dir.norm();
        return ((0.25 / 3.141592653589793238462643383) / (l * l * l)) * weightedNormalSum.dot(dir);
    }

    TraversalControl WindingNumberTraversalController::analyze(const AABBTreeNode &node, int nodeIndex)
    {
        if (node.isLeaf())
        {
            const unsigned int *tri = &mTriangles[3 * node.getPrimitiveIndex()];
            mWindingNumber += evaluateExact(mPoints[tri[0]], mPoints[tri[1]], mPoints[tri[2]], mQueryPoint);
            return TraversalControl::eDontGoDeeper;
        }
        const ClusterApproximation &cluster = mClusters.find(nodeIndex)->second;
        const float distSquared = (mQueryPoint - cluster.WeightedCentroid).squaredNorm();
        const float threshold = mDistanceThresholdBeta * cluster.Radius;
        if (distSquared > threshold * threshold)
        {
            mWindingNumber += firstOrderClusterApproximation(cluster.WeightedCentroid, cluster.WeightedNormalSum, mQueryPoint);
            return TraversalControl::eDontGoDeeper;
        }
        return TraversalControl::eGoDeeper;
    }
} // namespace crmpm
