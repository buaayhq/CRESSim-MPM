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

#ifndef CR_PARTICLE_OBJECT_IMPL_H
#define CR_PARTICLE_OBJECT_IMPL_H

#include "particle_object.h"
#include "ref_counted.h"
#include "array.h"
#include "vec3.h"
#include "quat.h"
#include "material_data.h"

namespace crmpm
{
    class Scene;
    class Geometry;
    class SimulationFactoryImpl;

    class ParticleObjectImpl : public ParticleObject, public RefCounted
    {
    public:
        /**
         * @brief Copy the particle object data to the scene. Once added, 
         * it cannot be removed from a scene.
         */
        bool addToScene(Scene &scene) override;

        unsigned int getIndexOffset() override { return mIndexOffset; }

        unsigned int getNumParticles() override { return mNumParticles; }

        /**
         * @brief Reset the particle group in the scene to its initial states.
         * 
         * This method does not have effect before the particle object has been
         * added to a scene.
         */
        void reset() override;

    protected:
        ParticleObjectImpl() : mInitialParticlePositionMass(1000) {}
        ~ParticleObjectImpl() {}

        void initialize(float particleSpacing,
                                  const Geometry &geom,
                                  const Vec3f &position,
                                  const Quat &rotation,
                                  const Vec3f &invScale);

        void setParticleMass(float mass)
        {
            mParticleMass = mass;
        }
    
        void setMaterial(const ParticleMaterialType &type, const float4 &params)
        {
            mParticleMaterialType = type;
            mParticleMaterialParams = params;
        }

    private:
        Scene *mScene = nullptr;

        // Particle physics properties
        float mParticleMass;
        float4 mParticleMaterialParams;
        ParticleMaterialType mParticleMaterialType;

        // Index offset in scene
        unsigned int mIndexOffset = 0;
        unsigned int mNumParticles = 0;

        // Stored initial particle data
        Array<float4> mInitialParticlePositionMass;

        void _release() override;

        friend class SimulationFactoryImpl;
    };
} // namespace crmpm


#endif // !CR_PARTICLE_OBJECT_IMPL_H