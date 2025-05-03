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

#ifndef CR_GEOMETRY_IMPL_H
#define CR_GEOMETRY_IMPL_H

#include "geometry.h"
#include "ref_counted.h"
#include "bounds3.h"
#include "gpu_data_dirty_flags.h"
#include "factory_controlled.h"

namespace crmpm
{
    class SimulationFactoryImpl;

    class GeometryImpl : public Geometry, public FactoryControlled, public RefCounted
    {
    public:
        CR_FORCE_INLINE int getId() const override { return mId; }

        CR_FORCE_INLINE GeometryType getType() const override { return mGeometryData->type[mId]; };

        CR_FORCE_INLINE void setParams(float4 params0) override
        {
            mGeometryData->params0[mId] = params0;
            markDirty(SimulationFactoryGpuDataDirtyFlags::eGeometryParams0);
        }

        CR_FORCE_INLINE const float4 &getParams() const override
        {
            return mGeometryData->params0[mId];
        }

        CR_FORCE_INLINE const Bounds3 &getBounds() const override { return mBounds; }

        void queryPointSdf(const Vec3f &point, float4 &sdfGradientDistance) const override;

    protected:
        GeometryImpl() {};

        ~GeometryImpl() {};

        CR_FORCE_INLINE void setId(int id) { mId = id; }

        CR_FORCE_INLINE void bindGeometryData(GeometryData &geometryData, GeometrySdfData &geometrySdfData)
        {
            mGeometryData = &geometryData;
            mGeometrySdfData = &geometrySdfData;
        }

        CR_FORCE_INLINE void setBounds(const Bounds3 &bounds)
        {
            mBounds = bounds;
        }

    private:
        int mId;

        // Linked geometry data
        GeometryData *mGeometryData;
        GeometrySdfData *mGeometrySdfData;

        // Bounds
        Bounds3 mBounds;

        void _release() override;

        friend class SimulationFactoryImpl;
    };
} // namespace crmpm

#endif // !CR_GEOMETRY_IMPL_H