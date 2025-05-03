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

#ifndef CR_PARTICLE_OBJECT_H
#define CR_PARTICLE_OBJECT_H

#include "releasable.h"
#include "engine_export.h"

namespace crmpm
{
    class Scene;

    class CRMPM_ENGINE_API ParticleObject : virtual public Releasable
    {
    public:
        /**
         * @brief Copy the particle object data to a `Scene`. Once added, 
         * it cannot be removed from the scene.
         */
        virtual bool addToScene(Scene &scene) = 0;

        /**
         * @brief Get the index offset of the particles in the Scene's
         * global particle array.
         */
        virtual unsigned int getIndexOffset() = 0;

        /**
         * @brief Get the total number of particles in this `ParticleObject`.
         */
        virtual unsigned int getNumParticles() = 0;

        /**
         * @brief Reset the particle group in the scene to its initial states.
         * This method does not have effect before the particle object has been
         * added to a scene.
         */
        virtual void reset() = 0;

    protected:
        ParticleObject() {}

        virtual ~ParticleObject() {}
    };
} // namespace crmpm


#endif // !CR_PARTICLE_OBJECT_H