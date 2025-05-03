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

#include "particle_object_impl.h"
#include "simulation_factory.h"
#include "scene_impl.h"

namespace crmpm
{
    void ParticleObjectImpl::initialize(
        float particleSpacing,
        const Geometry &geom,
        const Vec3f &position,
        const Quat &rotation,
        const Vec3f &invScale)
    {
        Bounds3 geometryBounds = geom.getBounds();
        geometryBounds.fattenFast(2 * particleSpacing);

        float startX = geometryBounds.minimum.x();
        float startY = geometryBounds.minimum.y();
        float startZ = geometryBounds.minimum.z();
        float endX = geometryBounds.maximum.x();
        float endY = geometryBounds.maximum.y();
        float endZ = geometryBounds.maximum.z();

        Vec3i particleQueryDimension = ((geometryBounds.maximum - geometryBounds.minimum) / particleSpacing / invScale).cast<int>();

        unsigned int numParticles = 0;

        for (int ix = 0; ix < particleQueryDimension.x(); ++ix)
        {
            for (int iy = 0; iy < particleQueryDimension.y(); ++iy)
            {
                for (int iz = 0; iz < particleQueryDimension.z(); ++iz)
                {
                    Vec3f queryPoint = Vec3f(ix, iy, iz) * particleSpacing * invScale + geometryBounds.minimum;
                    float4 sdfResult;

                    geom.queryPointSdf(queryPoint, sdfResult);
                    if (sdfResult.w <= 0)
                    {
                        // Convert to world position
                        Vec3f p = queryPoint / invScale;
                        p = rotation.rotate(p);
                        p += position;

                        // Assign position mass
                        float4 particle;
                        particle.x = p.x();
                        particle.y = p.y();
                        particle.z = p.z();
                        particle.w = mParticleMass;
                        mInitialParticlePositionMass.pushBack(particle);

                        numParticles++;
                    }
                }
            }
        }
        mNumParticles = numParticles;
    }

    bool ParticleObjectImpl::addToScene(Scene &scene)
    {
        if (mScene)
        {
            CR_DEBUG_LOG_WARNING("%s", "A ParticleObject can only be added to a Scene once. Skipped.");
            return false;
        }
        if (!scene.allocateParticles(mNumParticles, mIndexOffset))
        {
            CR_DEBUG_LOG_WARNING("%s", "No enough particle capacity in scene. Skipped.");
            return false;
        }

        mScene = &scene;
        static_cast<SceneImpl &>(scene).incrementRef();

        Array<unsigned int> &computeInitialDataIndices = scene.getComputeInitialDataIndices();
        ParticleData &particleData = scene.getParticleData();
        ParticleMaterialData &particleMaterialData = scene.getParticleMaterialData();

        memcpy(particleData.positionMass + mIndexOffset, mInitialParticlePositionMass.begin(), mNumParticles * sizeof(float4));
        for (unsigned int i = 0; i < mNumParticles; ++i)
        {
            unsigned int idx = mIndexOffset + i;

            // Material
            particleMaterialData.params0[idx] = mParticleMaterialParams;
            particleMaterialData.type[idx] = mParticleMaterialType;

            computeInitialDataIndices.pushBack(idx);
        }
        scene.setActiveMaskRange(mIndexOffset, mNumParticles, 1);

        scene.markDirty(SceneDataDirtyFlags::eNumParticles);
        scene.markDirty(SceneDataDirtyFlags::eParticlePositionMass);
        scene.markDirty(SceneDataDirtyFlags::eParticleMaterialParams0);
        scene.markDirty(SceneDataDirtyFlags::eParticleMaterialType);
        scene.markDirty(SceneDataDirtyFlags::eComputeInitialData);
        return true;
    }

    void ParticleObjectImpl::reset()
    {
        if (!mScene)
            return;

        Array<unsigned int> &computeInitialDataIndices = mScene->getComputeInitialDataIndices();
        ParticleData &particleData = mScene->getParticleData();
        ParticleMaterialData &particleMaterialData = mScene->getParticleMaterialData();

        memcpy(particleData.positionMass + mIndexOffset, mInitialParticlePositionMass.begin(), mNumParticles * sizeof(float4));
        for (unsigned int i = 0; i < mNumParticles; ++i)
        {
            unsigned int idx = mIndexOffset + i;

            // Material
            particleMaterialData.params0[idx] = mParticleMaterialParams;
            particleMaterialData.type[idx] = mParticleMaterialType;

            computeInitialDataIndices.pushBack(idx);
        }
        mScene->setActiveMaskRange(mIndexOffset, mNumParticles, 1);

        mScene->markDirty(SceneDataDirtyFlags::eParticlePositionMass);
        mScene->markDirty(SceneDataDirtyFlags::eParticleMaterialParams0);
        mScene->markDirty(SceneDataDirtyFlags::eParticleMaterialType);
        mScene->markDirty(SceneDataDirtyFlags::eComputeInitialData);
    }

    void ParticleObjectImpl::_release()
    {
        CR_DEBUG_LOG_INFO("%s", "Releasing ParticleObject.");
        if (mScene)
        {
            static_cast<SceneImpl *>(mScene)->release();
        }
    }
} // namespace crmpm
