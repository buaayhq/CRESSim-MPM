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

#include <cuda_runtime.h>

#include "scene_impl.h"
#include "shape_impl.h"
#include "check_cuda.cuh"
#include "simulation_factory_impl.h"
#include "grow_data_block.cuh"
#include "material_data.h"

namespace crmpm
{
    void SceneImpl::initialize(int maxNumParticles)
    {
        // Initialize a separate thread
        mThreadManager.start();

        if (mIsGpu)
        {
            // Use pinned host memory for particle position/mass and velocity
            // Material data should not be frequently changed so we don't use pinned memory for them
            CR_CHECK_CUDA(cudaMallocHost<float4>(&mParticlePositionMass, maxNumParticles * sizeof(float4)));
            CR_CHECK_CUDA(cudaMallocHost<Vec3f>(&mParticleVelocity, maxNumParticles * sizeof(Vec3f)));
        }
        else
        {
            mParticlePositionMass = new float4[maxNumParticles];
            mParticleVelocity = new Vec3f[maxNumParticles];
        }
        mCpuParticleData.positionMass = mParticlePositionMass;
        mCpuParticleData.velocity = mParticleVelocity;

        mParticleMaterialTypes = new ParticleMaterialType[maxNumParticles];
        mParticleMaterialPropertiesGroup1 = new float4[maxNumParticles];
        mCpuParticleMaterialData.params0 = mParticleMaterialPropertiesGroup1;
        mCpuParticleMaterialData.type = mParticleMaterialTypes;

        mActiveParticleMask = new unsigned char[maxNumParticles];

        // For GPU solvers
        if (mIsGpu)
        {
            CR_CHECK_CUDA(cudaMalloc<float4>(&dmParticlePositionMass, maxNumParticles * sizeof(float4)));
            CR_CHECK_CUDA(cudaMalloc<Vec3f>(&dmParticleVelocity, maxNumParticles * sizeof(Vec3f)));
            CR_CHECK_CUDA(cudaMalloc<float4>(&dmParticleMaterialPropertiesGroup1, maxNumParticles * sizeof(float4)));
            CR_CHECK_CUDA(cudaMalloc<ParticleMaterialType>(&dmParticleMaterialTypes, maxNumParticles * sizeof(ParticleMaterialType)));

            CR_CHECK_CUDA(cudaMemset(dmParticlePositionMass, 0, maxNumParticles * sizeof(float4)));
            CR_CHECK_CUDA(cudaMemset(dmParticleVelocity, 0, maxNumParticles * sizeof(Vec3f)));
            CR_CHECK_CUDA(cudaMemset(dmParticleMaterialPropertiesGroup1, 0, maxNumParticles * sizeof(float4)));
            CR_CHECK_CUDA(cudaMemset(dmParticleMaterialTypes, 0, maxNumParticles * sizeof(ParticleMaterialType)));

            mGpuParticleData.positionMass = dmParticlePositionMass;
            mGpuParticleData.velocity = dmParticleVelocity;
            mGpuParticleMaterialData.params0 = dmParticleMaterialPropertiesGroup1;
            mGpuParticleMaterialData.type = dmParticleMaterialTypes;

            CR_CHECK_CUDA(cudaMalloc<int>(&dmShapeIds, mShapeCapacity * sizeof(int)));
            CR_CHECK_CUDA(cudaMalloc<unsigned char>(&dmActiveParticleMask, maxNumParticles * sizeof(unsigned char)));
            CR_CHECK_CUDA(cudaMalloc<unsigned int>(&dmComputeInitialDataIndices, maxNumParticles * sizeof(int)));

            mSolver->bindParticleData(mGpuParticleData);
            mSolver->bindParticleMaterials(mGpuParticleMaterialData);
            mSolver->bindShapeIds(dmShapeIds);
            mSolver->bindActiveMask(dmActiveParticleMask);
        }
        else
        {
            mSolver->bindParticleData(mCpuParticleData);
            mSolver->bindParticleMaterials(mCpuParticleMaterialData);
            mSolver->bindShapeIds(mShapeIds.begin());
            mSolver->bindActiveMask(mActiveParticleMask);
        }

        mSolver->initialize();
    }

    void SceneImpl::advance(float dt)
    {
        if (mStatus == SimulationStatus::eIdle)
        {
            mThreadManager.submitTask(mAdvanceTaskFn, &dt, sizeof(float));
            mStatus = SimulationStatus::eBusy;
        }
        else
        {
            CR_DEBUG_LOG_WARNING("%s", "fetchResults() has not been called after advance().");
        }
    }

    void SceneImpl::_advanceTaskFn(void *data)
    {
        beforeAdvance();
        float dt = *((float *)data);
        mFactory->beforeSceneAdvance(); // Note: a second scene advance will not trigger copy again.
        float timeAdvanced = 0;
        while (timeAdvanced < dt)
        {
            timeAdvanced += mSolver->step();
        }

        mSolver->fetchResults();

        if (mIsGpu)
        {
            // GPU->CPU
            CR_CHECK_CUDA(cudaMemcpy(mCpuParticleData.positionMass, dmParticlePositionMass, mMaxNumParticles * sizeof(float4), cudaMemcpyKind::cudaMemcpyDeviceToHost));
            CR_CHECK_CUDA(cudaMemcpy(mCpuParticleData.velocity, dmParticleVelocity, mMaxNumParticles * sizeof(Vec3f), cudaMemcpyKind::cudaMemcpyDeviceToHost));
        }

        for (ShapeImpl *shape : mShapes)
        {
            shape->resetKinematicTarget();
        }

        // TODO: This will lead to copy every time a scene fetches results.
        // A better way is to have a Factory.advanceAll/fetchAll
        mFactory->afterSceneAdvance(mIsGpu);
    }

    void SceneImpl::setActiveMaskRange(unsigned int start, unsigned int length, char value)
    {
        if (mIsGpu)
        {
            CR_CHECK_CUDA(cudaMemsetAsync(dmActiveParticleMask + start, value, length))
        }

        // Always set host array in case user wants to read using getActiveMask().
        memset(mActiveParticleMask + start, value, length);
    }

    void SceneImpl::addShape(Shape *shape)
    {
        // Prevent adding a shape twice
        if (mShapeIds.find(shape->getId()) != mShapeIds.end())
        {
            CR_DEBUG_LOG_WARNING("%s", "Attempted to add the same shape to a scene twice. Discarded.");
            return;
        }

        // Grow shape capacity if needed
        if (mShapeIds.size() == mShapeCapacity)
        {
            int newCapacity = mShapeCapacity * 1.5 + 1;
            mShapeIds.reserve(newCapacity);

            if (mIsGpu)
            {
                growGpuData(dmShapeIds, mShapeCapacity, newCapacity);
                mSolver->bindShapeIds(dmShapeIds);
            }
            else
            {
                mSolver->bindShapeIds(mShapeIds.begin());
            }

            mShapeCapacity = newCapacity;
        }

        // Add to scene
        static_cast<ShapeImpl *>(shape)->incrementRef();
        mShapeIds.tryPushBack(shape->getId());
        mShapes.pushBack(static_cast<ShapeImpl *>(shape));
        if (mIsGpu)
        {
            CR_CHECK_CUDA(cudaMemcpy(dmShapeIds, mShapeIds.begin(), mShapeCapacity * sizeof(int), cudaMemcpyKind::cudaMemcpyHostToDevice));
        }
        mSolver->setNumShapes(mShapeIds.size());
    }

    void SceneImpl::syncDataIfNeeded()
    {
        if (mDataDirtyFlags == SceneDataDirtyFlags::eNone)
            return;

        if (mDataDirtyFlags & SceneDataDirtyFlags::eNumParticles)
        {
            mSolver->setNumActiveParticles(mNumAllocatedParticles);
        }

        if (mIsGpu && mDataDirtyFlags & SceneDataDirtyFlags::eParticlePositionMass)
            CR_CHECK_CUDA(cudaMemcpy(mGpuParticleData.positionMass, mCpuParticleData.positionMass, mMaxNumParticles * sizeof(float4), cudaMemcpyKind::cudaMemcpyHostToDevice));

        if (mIsGpu && mDataDirtyFlags & SceneDataDirtyFlags::eParticleVelocity)
            CR_CHECK_CUDA(cudaMemcpy(mGpuParticleData.velocity, mCpuParticleData.velocity, mMaxNumParticles * sizeof(Vec3f), cudaMemcpyKind::cudaMemcpyHostToDevice));

        if (mIsGpu && mDataDirtyFlags & SceneDataDirtyFlags::eParticleMaterialParams0)
            CR_CHECK_CUDA(cudaMemcpy(mGpuParticleMaterialData.params0, mCpuParticleMaterialData.params0, mMaxNumParticles * sizeof(float4), cudaMemcpyKind::cudaMemcpyHostToDevice));

        if (mIsGpu && mDataDirtyFlags & SceneDataDirtyFlags::eParticleMaterialType)
            CR_CHECK_CUDA(cudaMemcpy(mGpuParticleMaterialData.type, mCpuParticleMaterialData.type, mMaxNumParticles * sizeof(ParticleMaterialType), cudaMemcpyKind::cudaMemcpyHostToDevice));

        if (mDataDirtyFlags & SceneDataDirtyFlags::eComputeInitialData)
        {
            if (mComputeInitialDataIndices.size() > mMaxNumParticles)
            {
                CR_DEBUG_LOG_WARNING("%s", "Compute initial data particle num > max particle num!");
            }
            if (mIsGpu)
            {
                CR_CHECK_CUDA(cudaMemcpy(dmComputeInitialDataIndices, mComputeInitialDataIndices.begin(), mComputeInitialDataIndices.size() * sizeof(unsigned int), cudaMemcpyKind::cudaMemcpyHostToDevice));
                mSolver->computeInitialData(mComputeInitialDataIndices.size(),
                                            dmComputeInitialDataIndices);
            }
            else
            {
                mSolver->computeInitialData(mComputeInitialDataIndices.size(),
                                            mComputeInitialDataIndices.begin());
            }
            mComputeInitialDataIndices.clear();
        }

        mDataDirtyFlags = SceneDataDirtyFlags::eNone;
    }

    void SceneImpl::_release()
    {
        CR_DEBUG_LOG_INFO("%s", "Releasing Scene.");
        // In case solver still running
        if (mStatus == SimulationStatus::eBusy)
            fetchResults();
        
        // Release all referred shapes
        for (ShapeImpl *shape : mShapes)
        {
            shape->release();
        }
        mShapes.clear();

        mSolver->release();
        delete[] mParticleMaterialTypes;
        delete[] mParticleMaterialPropertiesGroup1;
        delete[] mActiveParticleMask;

        if (mIsGpu)
        {
            CR_CHECK_CUDA(cudaFreeHost(mParticlePositionMass));
            CR_CHECK_CUDA(cudaFreeHost(mParticleVelocity));

            CR_CHECK_CUDA(cudaFree(dmParticlePositionMass));
            CR_CHECK_CUDA(cudaFree(dmParticleVelocity));
            CR_CHECK_CUDA(cudaFree(dmParticleMaterialPropertiesGroup1));
            CR_CHECK_CUDA(cudaFree(dmParticleMaterialTypes));
            CR_CHECK_CUDA(cudaFree(dmShapeIds));
            CR_CHECK_CUDA(cudaFree(dmActiveParticleMask));
            CR_CHECK_CUDA(cudaFree(dmComputeInitialDataIndices));
        }
        else
        {
            delete[] mParticlePositionMass;
            delete[] mParticleVelocity;
        }
    }

    void SceneImpl::beforeAdvance()
    {
        // Set all shape data to be zero
        for (ShapeImpl *shape : mShapes)
        {
            shape->resetCouplingForce();
        }

        syncDataIfNeeded();
    }

} // namespace crmpm
