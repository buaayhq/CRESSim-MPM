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

#ifndef CR_GEOMETRY_H
#define CR_GEOMETRY_H

#include "bounds3.h"
#include "releasable.h"
#include "engine_export.h"

namespace crmpm
{
    enum class GeometryType : int
    {
        // Regular geometries

        eBox,
        eSphere,
        ePlane,
        eCapsule,
        eTriangleMesh,

        // Open boundary slicer geometries

        eQuadSlicer,
        eTriangleMeshSlicer,

        // Line & Arc

        eConnectedLineSegments,
        eArc,
    };

    /**
     * @brief Geometry data SOA.
    */
    struct GeometryData
    {
        GeometryType *type;

        /**
        *
        * **Box**
        * - `params0.xyz`: half size
        *
        * **Sphere**
        * - `params0.x`: radius
        *
        * **Plane**
        * - No parameters
        *
        * **Capsule**
        * - `params0.x`: radius
        * - `params0.y`: cylinder half height
        * 
        * **QuadSlicer**
        * - `params0.xy`: half size
        *
        * **SDF Geometries**
        * - `params0.x`: sdfDataOffset
        * - `params0.y`: sdfDataId
        * - `params0.z`: sdfDataSize
        * - `gradientSignedDistance`: first element slicer spine area
        *
        * **Connected Line Segments**
        * - Line points stored in sdf data: `gradientSignedDistance`
        * - `lowerBoundCellSize`: stores the lower bound
        * - `gradientSignedDistance`:  first element stores the upper bound
        * - `params0.x`: sdfDataOffset
        * - `params0.y`: sdfDataId
        * - `params0.z`: sdfDataSize
        *
        * **Arc**
        * - `params0.x`: angle lower bound (-pi, 0)
        * - `params0.y`: angle upper bound (0, pi)
        */
        float4 *params0;

        int size;
    };

    /**
     * @brief GeometrySdfData SOA.
     */
    struct GeometrySdfData
    {
        Vec3i *dimemsion = nullptr;               // SDF dimension, accessed using GeometryData.sdfDataId
        float4 *lowerBoundCellSize = nullptr;     // Fused lower bound + cell size of SDF
        float4 *gradientSignedDistance = nullptr; // Flattened SDF data for all geometry, accessed using GeometryData.sdfDataOffset
    };

    class CRMPM_ENGINE_API Geometry : virtual public Releasable
    {
    public:
        /**
         * @brief Get the global ID of the Geometry.
         */
        virtual int getId() const = 0;

        /**
         * @brief Get the geometry type.
         */
        virtual GeometryType getType() const = 0;

        /**
         * @brief Set the geometry parameters.
         * @param params0 Geometry parameters. See `GeometryData`.
         */
        virtual void setParams(float4 params0) = 0;

        /**
         * @brief Get the geometry parameters. See `GeometryData`.
         */
        virtual const float4 &getParams() const = 0;

        /**
         * @brief Get the geometry bounds.
         */
        virtual const Bounds3 &getBounds() const = 0;

        /**
         * @brief Query the signed distance to the geometry at a point.
         * @param[in] point The query point.
         * @param[out] sdfGradientDistance Output value of signed distance (last element) and gradient (first three).
         */
        virtual void queryPointSdf(const Vec3f &point, float4 &sdfGradientDistance) const = 0;

    protected:
        Geometry() {};

        virtual ~Geometry() {};
    };
} // namespace crmpm

#endif // !CR_GEOMETRY_H