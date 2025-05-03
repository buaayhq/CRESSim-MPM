// BSD 3-Clause License
//
// Copyright (c) 2025, Yafei Ou and Mahdi Tavakoli
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <cassert>

#include "vec3.h"
#include "sdf_builder.h"
#include "debug_logger.h"
#include "aabb_tree.h"
#include "winding_number.h"
#include "obj_loader.h"

// Helper function to generate test bounding volumes
void generateTestBounds(unsigned int nbBounds, crmpm::Array<crmpm::Bounds3> &bounds)
{
    bounds.resize(nbBounds);
    for (unsigned int i = 0; i < nbBounds; ++i)
    {
        crmpm::Bounds3 b;
        b.minimum = {i * 1.0f, i * 2.0f, i * 3.0f};
        b.maximum = {i * 1.0f + 1.0f, i * 2.0f + 1.0f, i * 3.0f + 1.0f};
        bounds[i] = b;
    }
}

// Test function
void testAABBTree()
{
    // Number of test bounds
    const unsigned int nbBounds = 10;

    // Generate test bounds
    CR_DEBUG_LOG_INFO("%s", "generate bounds");
    crmpm::Array<crmpm::Bounds3> testBounds;
    generateTestBounds(nbBounds, testBounds);

    // Allocate tree array
    crmpm::Array<crmpm::AABBTreeNode> tree;

    // Build the AABB tree
    CR_DEBUG_LOG_INFO("%s", "build tree");
    buildAABBTree(nbBounds, testBounds.begin(), tree);

    CR_DEBUG_LOG_INFO("%s", "check tree");
    // Validate the tree

    // Check root node
    const crmpm::AABBTreeNode &root = tree[0];

    // Traverse the tree and validate each node
    crmpm::Array<const crmpm::AABBTreeNode *> stack;
    stack.pushBack(&root);

    for (int i = 0; i < tree.size(); i++)
    {
        CR_DEBUG_LOG_INFO("Node %d", i);
        crmpm::AABBTreeNode &node = tree[i];

        CR_DEBUG_LOG_INFO("%f", node.mBV.minimum.x());
        CR_DEBUG_LOG_INFO("%f", node.mBV.minimum.y());
        CR_DEBUG_LOG_INFO("%f", node.mBV.minimum.z());

        CR_DEBUG_LOG_INFO("%f", node.mBV.maximum.x());
        CR_DEBUG_LOG_INFO("%f", node.mBV.maximum.y());
        CR_DEBUG_LOG_INFO("%f", node.mBV.maximum.z());
    }

    while (!stack.empty())
    {
        const crmpm::AABBTreeNode *node = stack.back();
        stack.popBack();

        if (node->isLeaf())
        {
            // Validate leaf node
            unsigned int nbPrimitives = node->getNbPrimitives();
            CR_DEBUG_LOG_INFO("Leaf node primitives: %d", nbPrimitives);

            unsigned int primitiveIndex = node->getPrimitiveIndex();
            CR_DEBUG_LOG_INFO("Primitive index: %d", primitiveIndex);

            const crmpm::Bounds3 &leafBounds = testBounds[primitiveIndex];
            CR_DEBUG_LOG_INFO("%s", leafBounds.minimum.x() >= node->mBV.minimum.x() ? "Leaf BV min x: ok" : "Leaf BV min x: error");
            CR_DEBUG_LOG_INFO("%s", leafBounds.maximum.x() <= node->mBV.maximum.x() ? "Leaf BV max x: ok" : "Leaf BV max x: error");
            CR_DEBUG_LOG_INFO("%s", leafBounds.minimum.y() >= node->mBV.minimum.y() ? "Leaf BV min y: ok" : "Leaf BV min y: error");
            CR_DEBUG_LOG_INFO("%s", leafBounds.maximum.y() <= node->mBV.maximum.y() ? "Leaf BV max y: ok" : "Leaf BV max y: error");
            CR_DEBUG_LOG_INFO("%s", leafBounds.minimum.z() >= node->mBV.minimum.z() ? "Leaf BV min z: ok" : "Leaf BV min z: error");
            CR_DEBUG_LOG_INFO("%s", leafBounds.maximum.z() <= node->mBV.maximum.z() ? "Leaf BV max z: ok" : "Leaf BV max z: error");
        }
        else
        {
            // Validate internal node
            const crmpm::AABBTreeNode *posChild = node->getPos(tree.begin());
            const crmpm::AABBTreeNode *negChild = node->getNeg(tree.begin());

            CR_DEBUG_LOG_INFO("Positive child existence %s", posChild != nullptr ? "ok" : "error");
            CR_DEBUG_LOG_INFO("Negative child existence %s", negChild != nullptr ? "ok" : "error");

            CR_DEBUG_LOG_INFO("%f", node->mBV.minimum.x());
            CR_DEBUG_LOG_INFO("%f", node->mBV.minimum.y());
            CR_DEBUG_LOG_INFO("%f", node->mBV.minimum.z());

            CR_DEBUG_LOG_INFO("%f", node->mBV.maximum.x());
            CR_DEBUG_LOG_INFO("%f", node->mBV.maximum.y());
            CR_DEBUG_LOG_INFO("%f", node->mBV.maximum.z());

            if (posChild && negChild)
            {
                // Push children to the stack
                stack.pushBack(posChild);
                stack.pushBack(negChild);

                // Validate bounding volume hierarchy
                CR_DEBUG_LOG_INFO("%s", posChild->mBV.minimum.x() >= node->mBV.minimum.x() ? "Pos child BV min x: ok" : "Pos child BV min x: error");
                CR_DEBUG_LOG_INFO("%s", posChild->mBV.maximum.x() <= node->mBV.maximum.x() ? "Pos child BV max x: ok" : "Pos child BV max x: error");
                CR_DEBUG_LOG_INFO("%s", negChild->mBV.minimum.x() >= node->mBV.minimum.x() ? "Neg child BV min x: ok" : "Neg child BV min x: error");
                CR_DEBUG_LOG_INFO("%s", negChild->mBV.maximum.x() <= node->mBV.maximum.x() ? "Neg child BV max x: ok" : "Neg child BV max x: error");

                CR_DEBUG_LOG_INFO("%s", posChild->mBV.minimum.y() >= node->mBV.minimum.y() ? "Pos child BV min y: ok" : "Pos child BV min y: error");
                CR_DEBUG_LOG_INFO("%s", posChild->mBV.maximum.y() <= node->mBV.maximum.y() ? "Pos child BV max y: ok" : "Pos child BV max y: error");
                CR_DEBUG_LOG_INFO("%s", negChild->mBV.minimum.y() >= node->mBV.minimum.y() ? "Neg child BV min y: ok" : "Neg child BV min y: error");
                CR_DEBUG_LOG_INFO("%s", negChild->mBV.maximum.y() <= node->mBV.maximum.y() ? "Neg child BV max y: ok" : "Neg child BV max y: error");

                CR_DEBUG_LOG_INFO("%s", posChild->mBV.minimum.z() >= node->mBV.minimum.z() ? "Pos child BV min z: ok" : "Pos child BV min z: error");
                CR_DEBUG_LOG_INFO("%s", posChild->mBV.maximum.z() <= node->mBV.maximum.z() ? "Pos child BV max z: ok" : "Pos child BV max z: error");
                CR_DEBUG_LOG_INFO("%s", negChild->mBV.minimum.z() >= node->mBV.minimum.z() ? "Neg child BV min z: ok" : "Neg child BV min z: error");
                CR_DEBUG_LOG_INFO("%s", negChild->mBV.maximum.z() <= node->mBV.maximum.z() ? "Neg child BV max z: ok" : "Neg child BV max z: error");
            }
        }
    }

    CR_DEBUG_LOG_INFO("%s", "Tree validation completed");
}

void testWindingNumberSimple()
{
    // crmpm::TriangleMesh mesh;
    crmpm::Vec3f3 *verts = new crmpm::Vec3f3[8];

    // Vertex positions
    verts[0] = {1.0f, 1.0f, -1.0f};   // Vertex 1
    verts[1] = {1.0f, -1.0f, -1.0f};  // Vertex 2
    verts[2] = {1.0f, 1.0f, 1.0f};    // Vertex 3
    verts[3] = {1.0f, -1.0f, 1.0f};   // Vertex 4
    verts[4] = {-1.0f, 1.0f, -1.0f};  // Vertex 5
    verts[5] = {-1.0f, -1.0f, -1.0f}; // Vertex 6
    verts[6] = {-1.0f, 1.0f, 1.0f};   // Vertex 7
    verts[7] = {-1.0f, -1.0f, 1.0f};  // Vertex 8

    // Triangle indices based on faces from the OBJ file
    unsigned int triangles[] = {
        // Face 1: 5, 3, 1
        4, 2, 0,
        // Face 2: 3, 8, 4
        2, 7, 3,
        // Face 3: 7, 6, 8
        6, 5, 7,
        // Face 4: 2, 8, 6
        1, 7, 5,
        // Face 5: 1, 4, 2
        0, 3, 1,
        // Face 6: 5, 2, 6
        4, 1, 5,
        // Face 7: 5, 7, 3
        4, 6, 2,
        // Face 8: 3, 7, 8
        2, 6, 7,
        // Face 9: 7, 5, 6
        6, 4, 5,
        // Face 10: 2, 4, 8
        1, 3, 7,
        // Face 11: 1, 3, 4
        0, 2, 3,
        // Face 12: 5, 1, 2
        4, 0, 1};

    // Test point
    crmpm::Vec3f3 p(0.5f, 0.5f, 0.5f);
    float windingNumberExact = computeWindingNumberExact(p, triangles, 12, verts);
    CR_DEBUG_LOG_INFO("%f", windingNumberExact);

    crmpm::Array<crmpm::AABBTreeNode> tree;
    buildTree(triangles, 12, verts, tree);

    crmpm::HashMap<unsigned int, crmpm::ClusterApproximation> clusters;
    precomputeClusterInformation(tree.begin(), triangles, 12, verts, clusters);
    float windingNumberApprox = computeWindingNumberApprox(tree.begin(), p, 2.0f, clusters, triangles, verts);
    CR_DEBUG_LOG_INFO("%f", windingNumberApprox);
}

void testWindingNumberObj()
{
    ObjLoader loader;
    loader.load("hemisphere.obj"); // Replace with your file path
    CR_DEBUG_LOG_INFO("%s", "load model ok");

    const int numTriangles = loader.getTriangles().size() / 3;
    const crmpm::Vec3f3 *verts = loader.getVertices().data();
    const unsigned int *triangles = loader.getTriangles().data();

    // Test point
    crmpm::Vec3f3 p(1.0f, -1.1f, 0.0f);
    float windingNumberExact = computeWindingNumberExact(p, triangles, numTriangles, verts);
    CR_DEBUG_LOG_INFO("%f", windingNumberExact);

    crmpm::Array<crmpm::AABBTreeNode> tree;
    buildTree(triangles, numTriangles, verts, tree);

    crmpm::HashMap<unsigned int, crmpm::ClusterApproximation> clusters;
    precomputeClusterInformation(tree.begin(), triangles, numTriangles, verts, clusters);
    float windingNumberApprox = computeWindingNumberApprox(tree.begin(), p, 2.0f, clusters, triangles, verts);
    CR_DEBUG_LOG_INFO("%f", windingNumberApprox);
}

void testComputeSdf()
{
    ObjLoader loader;
    loader.load("sphere.obj"); // Replace with your file path
    CR_DEBUG_LOG_INFO("%s", "load model ok");

    int numPoints = loader.getVertices().size();
    int numTriangles = loader.getTriangles().size() / 3;
    crmpm::Vec3f3 *verts = loader.getVertices().data();
    unsigned int *triangles = loader.getTriangles().data();

    crmpm::TriangleMesh mesh;
    mesh.points = verts;
    mesh.numPoints = numPoints;
    mesh.triangles = triangles;
    mesh.numTriangles = numTriangles;

    crmpm::SdfDesc sdfDesc;
    sdfDesc.cellSize = 0.2;
    sdfDesc.fatten = 0.05;

    crmpm::SdfBuilder::precomputeSdfDesc(sdfDesc, mesh);
    CR_DEBUG_LOG_INFO("%s", "compute sdf desc ok");
    float4 *gradientDistance = new float4[sdfDesc.sdfDimension.x() * sdfDesc.sdfDimension.y() * sdfDesc.sdfDimension.z()];

    crmpm::SdfBuilder::computeSdf(sdfDesc, mesh, gradientDistance);
    CR_DEBUG_LOG_INFO("%s", "compute sdf ok");

    crmpm::SdfBuilder::computeSdfGradient(sdfDesc, gradientDistance);
    CR_DEBUG_LOG_INFO("%s", "compute sdf gradient ok");

    crmpm::Vec3f sdfQueryPoint(0.5, 0, 0);
    crmpm::Vec3i mappedQueryPoint = ((sdfQueryPoint - sdfDesc.lowerBound) / sdfDesc.cellSize + crmpm::Vec3f(0.5f)).cast<int>(); // Add 0.5 then cast down results in the closest node. But of cource interpolation can be used.
    int index = mappedQueryPoint.z() * sdfDesc.sdfDimension.x() * sdfDesc.sdfDimension.y() + mappedQueryPoint.y() * sdfDesc.sdfDimension.x() + mappedQueryPoint.x();

    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].x);
    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].y);
    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].z);
    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].w);
}

void testComputeModSdf()
{
    ObjLoader loader;
    loader.load("plane.obj"); // Replace with your file path
    CR_DEBUG_LOG_INFO("%s", "load model ok");

    int numPoints = loader.getVertices().size();
    int numTriangles = loader.getTriangles().size() / 3;
    crmpm::Vec3f3 *verts = loader.getVertices().data();
    unsigned int *triangles = loader.getTriangles().data();

    crmpm::TriangleMesh mesh;
    mesh.points = verts;
    mesh.numPoints = numPoints;
    mesh.triangles = triangles;
    mesh.numTriangles = numTriangles;

    crmpm::SdfDesc sdfDesc;
    sdfDesc.cellSize = 0.05;
    sdfDesc.fatten = 2.0;

    crmpm::SdfBuilder::precomputeSdfDesc(sdfDesc, mesh);
    CR_DEBUG_LOG_INFO("%s", "compute sdf desc ok");
    float4 *gradientDistance = new float4[sdfDesc.sdfDimension.x() * sdfDesc.sdfDimension.y() * sdfDesc.sdfDimension.z()];

    crmpm::Array<bool> projectionOnMeshList;

    crmpm::SdfBuilder::computeModSdf(sdfDesc, mesh, gradientDistance, projectionOnMeshList);
    CR_DEBUG_LOG_INFO("%s", "compute sdf ok");

    crmpm::SdfBuilder::computeSdfGradient(sdfDesc, gradientDistance);
    crmpm::SdfBuilder::postProcessModSdfGradient(sdfDesc, gradientDistance, projectionOnMeshList);
    CR_DEBUG_LOG_INFO("%s", "compute sdf gradient ok");

    crmpm::Vec3f sdfQueryPoint(0.5, -0.5, 1.05);
    crmpm::Vec3i mappedQueryPoint = ((sdfQueryPoint - sdfDesc.lowerBound) / sdfDesc.cellSize + crmpm::Vec3f(0.5f)).cast<int>(); // Add 0.5 then cast down results in the closest node. But of cource interpolation can be used.
    int index = mappedQueryPoint.z() * sdfDesc.sdfDimension.x() * sdfDesc.sdfDimension.y() + mappedQueryPoint.y() * sdfDesc.sdfDimension.x() + mappedQueryPoint.x();

    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].x);
    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].y);
    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].z);
    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].w);

    sdfQueryPoint = crmpm::Vec3f(0.5, -0.5, 0);
    mappedQueryPoint = ((sdfQueryPoint - sdfDesc.lowerBound) / sdfDesc.cellSize + crmpm::Vec3f(0.5f)).cast<int>(); // Add 0.5 then cast down results in the closest node. But of cource interpolation can be used.
    index = mappedQueryPoint.z() * sdfDesc.sdfDimension.x() * sdfDesc.sdfDimension.y() + mappedQueryPoint.y() * sdfDesc.sdfDimension.x() + mappedQueryPoint.x();

    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].x);
    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].y);
    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].z);
    CR_DEBUG_LOG_INFO("%f", gradientDistance[index].w);
}

int main()
{
    // testAABBTree();
    testWindingNumberSimple();
    testWindingNumberObj();
    testComputeSdf();
    testComputeModSdf();
    return 0;
}