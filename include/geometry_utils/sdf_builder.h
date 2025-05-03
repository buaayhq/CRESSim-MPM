/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Yafei Ou and Mahdi Tavakoli
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CR_SDF_BUILDER_H
#define CR_SDF_BUILDER_H

#include "vec3.h"
#include "triangle_mesh.h"
#include "aabb_tree.h"
#include "winding_number.h"
#include "debug_logger.h"
#include "engine_export.h"

namespace crmpm
{
    /**
     * @brief Sdf description
     */
    struct SdfDesc
    {
        // User only need to specify these

        float cellSize = 0.1f; // The cell size for the discrete SDF voxels
        float fatten = 0;     // Distance to fatten the computed box region of SDF

        // A plane for slicer's spine area; (x, y, z) is the normal and w is the distance.
        // The normal's direction is the spine area.
        float4 slicerSpineArea = make_float4(nanf(""), 0, 0, 0); 

        // Computed data after precomputeSdfDesc

        Vec3f boundingSize; // The size of the bounding box to compute for SDF
        Vec3f lowerBound;   // The Lower bound of the bounding box to compute for SDF
        Vec3i sdfDimension; // Dimensions for the SDF box region.

        Array<AABBTreeNode> *tree;
        HashMap<unsigned int, ClusterApproximation> *clusters;
    };

    class CRMPM_ENGINE_API SdfBuilder
    {
    public:
        static void precomputeSdfDesc(SdfDesc &d, TriangleMesh &mesh);

        /**
         * @brief Compute the SDF data for a triangle mesh according to SdfDesc
         * 
         * @param[in] desc SDF description.
         * @param[out] gradientDistance
         */
        static void computeSdf(SdfDesc &d, TriangleMesh &mesh, float4 *gradientDistance);

        /**
         * @brief Compute the Modifed SDF (unsigned with side information) data for a triangle mesh according to SdfDesc.
         * 
         * @param[in] desc SDF description.
         * @param[out] gradientDistance
         */
        static void computeModSdf(SdfDesc &d, TriangleMesh &mesh, float4 *gradientDistance, Array<bool> &projectionOnMeshList);

        /**
         * Calculate SDF gradient from SDF disntance grid
         * 
         * @param[in] d SDF description.
         * @param[inout] gradientDistance the array with signed distance calculated.
         * Computed gradient will be written to the x, y, z components.
         */
        static void computeSdfGradient(SdfDesc &d, float4 *gradientDistance, bool normalize = true);

        static void postProcessModSdfGradient(SdfDesc &d, float4 *gradientDistance, Array<bool> &projectionOnMeshList);
    };
} // namespace crmpm

#endif