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

#include "sdf_builder.h"

namespace crmpm
{
    static void buildEdgeTriangleMap(const unsigned int *triangles, const Vec3f3 *points, float numTriangles, HashMap<Pair<int, int>, Array<int>, EdgeHash> &edgeTriangleMap)
    {
        for (int i = 0; i < numTriangles; i++)
        {
            const unsigned int *tri = &triangles[3 * i];
            for (int j = 0; j < 3; j++)
            {
                int v1 = tri[j];
                int v2 = tri[(j + 1) % 3];                      // Next vertex in triangle
                Pair<int, int> edge = std::minmax(v1, v2);      // Ensure ordering (v1, v2) is always sorted

                // Add the triangle index to the edge map
                edgeTriangleMap[edge].pushBack(i);
            }
        }
    }

    static void buildEdgeVerticesList(const HashMap<Pair<int, int>, Array<int>, EdgeHash> &edgeTriangleMap, Array<int> &edgeVertices)
    {
        for (const auto &edge : edgeTriangleMap)
        {
            if (edge.second.size() < 2)
            {
                const int vert1 = edge.first.first;
                const int vert2 = edge.first.second;

                if (edgeVertices.find(vert1) == edgeVertices.end())
                {
                    edgeVertices.pushBack(vert1);
                }
                if (edgeVertices.find(vert2) == edgeVertices.end())
                {
                    edgeVertices.pushBack(vert2);
                }
            }
        }
    }

    void SdfBuilder::precomputeSdfDesc(SdfDesc &d, TriangleMesh &mesh)
    {
        // Build AABB tree
        Array<AABBTreeNode> *tree = new Array<AABBTreeNode>();
        buildTree(mesh.triangles, mesh.numTriangles, mesh.points, *tree);

        HashMap<unsigned int, ClusterApproximation> *clusters = new HashMap<unsigned int, ClusterApproximation>();
        precomputeClusterInformation(tree->begin(), mesh.triangles, mesh.numTriangles, mesh.points, *clusters);

        d.tree = tree;
        d.clusters = clusters;

        Bounds3 sdfRegionBound = tree->begin()->mBV;
        sdfRegionBound.fattenFast(d.fatten);
        d.lowerBound = sdfRegionBound.minimum;
        d.boundingSize = (sdfRegionBound.maximum - sdfRegionBound.minimum);

        // Calculate sdfDimemsion
        d.sdfDimension = (d.boundingSize / d.cellSize + 1.0f).cast<int>();
    }

    void SdfBuilder::computeSdf(SdfDesc &d, TriangleMesh &mesh, float4 *gradientDistance)
    {
        int lastTriangle = -1;

        for (int z = 0; z < d.sdfDimension.z(); ++z)
        {
            for (int y = 0; y < d.sdfDimension.y(); ++y)
            {
                for (int x = 0; x < d.sdfDimension.x(); ++x)
                {
                    const int index = z * d.sdfDimension.x() * d.sdfDimension.y() + y * d.sdfDimension.x() + x;

                    // Get the query point for the current grid cell
                    Vec3f3 queryPoint = d.lowerBound.cast<float, 3>() + Vec3f3(x, y, z) * d.cellSize;

                    // Compute the closest distance to the mesh
                    ClosestDistanceToTrimeshTraversalController cd(mesh.triangles, mesh.points, d.tree->begin());
                    cd.setQueryPoint(queryPoint);

                    if (lastTriangle != -1)
                    {
                        // Warm-start the closest point search with the previous triangle
                        int i0 = mesh.triangles[3 * lastTriangle];
                        int i1 = mesh.triangles[3 * lastTriangle + 1];
                        int i2 = mesh.triangles[3 * lastTriangle + 2];

                        float t1, t2;
                        Vec3f3 q = queryPoint;
                        Vec3f3 a = mesh.points[i0];
                        Vec3f3 b = mesh.points[i1];
                        Vec3f3 c = mesh.points[i2];
                        Vec3f3 cp;
                        float d2 = distancePointTriangleSquared2UnitBox(q, a, b, c, t1, t2, cp);

                        cd.setClosestStart(d2, lastTriangle, cp);
                    }

                    traverseBVH(d.tree->begin(), cd);
                    Vec3f3 closestPoint = cd.getClosestPoint();
                    float closestDistance = (closestPoint - queryPoint).norm();

                    // Update the last triangle for warm-starting
                    lastTriangle = cd.getClosestTriId();

                    // Determine the sign of the distance (inside or outside)
                    float sign = 1.0f;
                    float windingNumber = computeWindingNumberApprox(d.tree->begin(), queryPoint, 2.0f, *d.clusters, mesh.triangles, mesh.points);
                    if (windingNumber > 0.5f)
                    {
                        sign = -1.0f; // Inside the mesh
                    }

                    // Store the signed distance in the SDF array
                    gradientDistance[index].w = closestDistance * sign;
                }
            }
        }
    }

    void SdfBuilder::computeModSdf(SdfDesc &d, TriangleMesh &mesh, float4 *gradientDistance, Array<bool> &projectionOnMeshList)
    {
        int lastTriangle = -1;
        projectionOnMeshList.clear();
        projectionOnMeshList.forceSizeUnsafe(d.sdfDimension.x() * d.sdfDimension.y() * d.sdfDimension.z());

        HashMap<Pair<int, int>, Array<int>, EdgeHash> edgeTriangleMap = HashMap<Pair<int, int>, Array<int>, EdgeHash>();
        Array<int> edgeVertList = Array<int>();
        buildEdgeTriangleMap(mesh.triangles, mesh.points, mesh.numTriangles, edgeTriangleMap);
        buildEdgeVerticesList(edgeTriangleMap, edgeVertList);

        for (int z = 0; z < d.sdfDimension.z(); ++z)
        {
            for (int y = 0; y < d.sdfDimension.y(); ++y)
            {
                for (int x = 0; x < d.sdfDimension.x(); ++x)
                {
                    const int index = z * d.sdfDimension.x() * d.sdfDimension.y() + y * d.sdfDimension.x() + x;

                    // Get the query point for the current grid cell
                    Vec3f3 queryPoint = d.lowerBound.cast<float, 3>() + Vec3f3(x, y, z) * d.cellSize;

                    // Compute the closest distance to the mesh
                    ClosestDistanceProjectionToTrimeshTraversalController cd(mesh.triangles, mesh.points, d.tree->begin(), &edgeTriangleMap, &edgeVertList);
                    cd.setQueryPoint(queryPoint);

                    if (lastTriangle != -1)
                    {
                        // Warm-start the closest point search with the previous triangle
                        int i0 = mesh.triangles[3 * lastTriangle];
                        int i1 = mesh.triangles[3 * lastTriangle + 1];
                        int i2 = mesh.triangles[3 * lastTriangle + 2];

                        float t1, t2;
                        Vec3f3 q = queryPoint;
                        Vec3f3 a = mesh.points[i0];
                        Vec3f3 b = mesh.points[i1];
                        Vec3f3 c = mesh.points[i2];
                        Vec3f3 cp;
                        float d2 = distancePointTriangleSquared2UnitBox(q, a, b, c, t1, t2, cp);

                        cd.setClosestStart(d2, lastTriangle, cp);
                    }

                    traverseBVH(d.tree->begin(), cd);
                    Vec3f3 closestPoint = cd.getClosestPoint();
                    float closestDistance = (closestPoint - queryPoint).norm();

                    // Update the last triangle for warm-starting
                    lastTriangle = cd.getClosestTriId();

                    // Store distance * side
                    int side = cd.getSide();
                    gradientDistance[index].w = closestDistance * side;
                    gradientDistance[index].w = fabsf(gradientDistance[index].w);

                    // bool projectionOnMesh = cd.getProjectionOnSurface();
                    projectionOnMeshList[index] = cd.getProjectionOnSurface();

                    // Set projectionOnMesh to be true if in spine area
                    if (isnan(d.slicerSpineArea.x))
                    {
                        continue;
                    }
                    float spineSide = queryPoint.dot(Vec3f3(d.slicerSpineArea.x, d.slicerSpineArea.y, d.slicerSpineArea.z).getNormalized()) + d.slicerSpineArea.w;
                    if (spineSide > 0)
                    {
                        projectionOnMeshList[index] = true;
                    }
                }
            }
        }
    }

    void SdfBuilder::computeSdfGradient(SdfDesc &d, float4 *gradientDistance, bool normalize)
    {
        // Extract dimensions
        int dimX = d.sdfDimension.x();
        int dimY = d.sdfDimension.y();
        int dimZ = d.sdfDimension.z();
        float cellSizeInv = 1.0f / d.cellSize; // Inverse of cell size

        // Lambda to safely access elements in the 3D array
        auto getIndex = [dimX, dimY](int x, int y, int z) -> int
        {
            return z * dimX * dimY + y * dimX + x;
        };

        // Loop over all points in the grid
        for (int z = 0; z < dimZ; ++z)
        {
            for (int y = 0; y < dimY; ++y)
            {
                for (int x = 0; x < dimX; ++x)
                {
                    // Compute gradient components
                    float dx = 0.0f, dy = 0.0f, dz = 0.0f;

                    // Handle X direction
                    if (x > 0 && x < dimX - 1)
                    {
                        dx = (gradientDistance[getIndex(x + 1, y, z)].w -
                              gradientDistance[getIndex(x - 1, y, z)].w) *
                             0.5f * cellSizeInv;
                    }
                    else if (x == 0)
                    {
                        dx = (gradientDistance[getIndex(x + 1, y, z)].w -
                              gradientDistance[getIndex(x, y, z)].w) *
                             cellSizeInv;
                    }
                    else if (x == dimX - 1)
                    {
                        dx = (gradientDistance[getIndex(x, y, z)].w -
                              gradientDistance[getIndex(x - 1, y, z)].w) *
                             cellSizeInv;
                    }

                    // Handle Y direction
                    if (y > 0 && y < dimY - 1)
                    {
                        dy = (gradientDistance[getIndex(x, y + 1, z)].w -
                              gradientDistance[getIndex(x, y - 1, z)].w) *
                             0.5f * cellSizeInv;
                    }
                    else if (y == 0)
                    {
                        dy = (gradientDistance[getIndex(x, y + 1, z)].w -
                              gradientDistance[getIndex(x, y, z)].w) *
                             cellSizeInv;
                    }
                    else if (y == dimY - 1)
                    {
                        dy = (gradientDistance[getIndex(x, y, z)].w -
                              gradientDistance[getIndex(x, y - 1, z)].w) *
                             cellSizeInv;
                    }

                    // Handle Z direction
                    if (z > 0 && z < dimZ - 1)
                    {
                        dz = (gradientDistance[getIndex(x, y, z + 1)].w -
                              gradientDistance[getIndex(x, y, z - 1)].w) *
                             0.5f * cellSizeInv;
                    }
                    else if (z == 0)
                    {
                        dz = (gradientDistance[getIndex(x, y, z + 1)].w -
                              gradientDistance[getIndex(x, y, z)].w) *
                             cellSizeInv;
                    }
                    else if (z == dimZ - 1)
                    {
                        dz = (gradientDistance[getIndex(x, y, z)].w -
                              gradientDistance[getIndex(x, y, z - 1)].w) *
                             cellSizeInv;
                    }

                    Vec3f3 normal(dx, dy, dz);
                    if (normalize)
                    {
                        normal.normalize();
                    }

                    // if (isnan(dx) || isnan(dy) || isnan(dx))
                    // {
                    //     normal.x() = nanf("0");
                    //     normal.y() = nanf("0");
                    //     normal.z() = nanf("0");
                    // }

                    // Write gradient to the array
                    gradientDistance[getIndex(x, y, z)] = make_float4(normal.x(), normal.y(), normal.z(), gradientDistance[getIndex(x, y, z)].w);
                }
            }
        }
    }

    void SdfBuilder::postProcessModSdfGradient(SdfDesc &d, float4 *gradientDistance, Array<bool> &projectionOnMeshList)
    {
        const int dimX = d.sdfDimension.x();
        const int dimY = d.sdfDimension.y();
        const int dimZ = d.sdfDimension.z();

        const int gridSize = dimX * dimY * dimZ;

        for (int i = 0; i < gridSize; ++i)
        {
            bool onMesh = projectionOnMeshList[i];
            if (!onMesh)
            {
                gradientDistance[i].x = nanf("0");
                gradientDistance[i].y = nanf("0");
                gradientDistance[i].z = nanf("0");
                gradientDistance[i].w = nanf("0");
            }
            // else if (side < 0)
            // {
            //     gradientDistance[i].x = -gradientDistance[i].x;
            //     gradientDistance[i].y = -gradientDistance[i].y;
            //     gradientDistance[i].z = -gradientDistance[i].z;
            // }
        }
    }
} // namespace crmpm
