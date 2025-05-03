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

#ifndef CR_SDF_H
#define CR_SDF_H

#include "preprocessor.h"
#include "math_helpers.h"
#include "vec3.h"
#include "geometry.h"
#include "type_punning.h"
#include "constants.h"

namespace crmpm
{
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE int getGridIndex(
        const Vec3i &dim,
        const int offset,
        const int x, const int y, const int z)
    {
        return offset + (z * dim.x() * dim.y()) + (y * dim.x()) + x;
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE int getGridIndex(
        const Vec3i &dim,
        const int offset,
        const Vec3i coord)
    {
        return offset + (coord.z() * dim.x() * dim.y()) + (coord.y() * dim.x()) + coord.x();
    }

    /**
     * Compute geometry SDF.
     * Analytical solutions for simple geometries. Referred to https://iquilezles.org/articles/distgradfunctions2d/.
     * Interpolating from pre-computed SDF data for complex shapes.
     * 
     * @returns For Slicer, whether the point is in the spine region.
     */
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE bool getGeometrySdf(
        const Vec3f localPosition,
        const GeometryType type,
        const float4 params0,
        const Vec3i *CR_RESTRICT sdfDimension,
        const float4 *CR_RESTRICT sdfLowerBoundCellSize,
        const float4 *CR_RESTRICT sdfGridData,
        float4 &sdfGradientDistance,
        Vec3f *tangentDirection = nullptr) // This is only used for ConnectedLineSegments
    {
        bool inSpine = false;
        switch (type)
        {
        case GeometryType::eBox:
        {
            Vec3f penetration = abs(localPosition) - params0;

            Vec3f s = Vec3f(localPosition.x() < 0.0 ? -1.0 : 1.0,
                            localPosition.y() < 0.0 ? -1.0 : 1.0,
                            localPosition.z() < 0.0 ? -1.0 : 1.0);

            float g = fmaxf(penetration.x(), fmaxf(penetration.y(), penetration.z())); // Inside distance
            Vec3f q = max(penetration, 0.0f);
            float l = q.norm(); // Outside distance

            // Gradient
            sdfGradientDistance = (s * ((g > 0.0) ? q / l : ((penetration.x() > penetration.y()) ? ((penetration.x() > penetration.z()) ? Vec3f(1, 0, 0) : Vec3f(0, 0, 1)) : ((penetration.y() > penetration.z()) ? Vec3f(0, 1, 0) : Vec3f(0, 0, 1))))).data;

            // Signed distance
            sdfGradientDistance.w = l + fminf(g, 0.0f);
            break;
        }

        case GeometryType::eSphere:
        {
            const float radius = params0.x;
            float length = localPosition.norm();
            const Vec3f gradient = localPosition / fmaxf(length, CR_EPS);

            sdfGradientDistance = gradient.data;
            sdfGradientDistance.w = length - radius;
            break;
        }

        case GeometryType::ePlane:
        {
            // A plane is a half-space volume y<=0
            sdfGradientDistance.x = 0.0f;
            sdfGradientDistance.y = 1.0f;
            sdfGradientDistance.z = 0.0f;
            sdfGradientDistance.w = localPosition.y();
            break;
        }

        case GeometryType::eCapsule:
        {
            /* A capsule's symmetry axis is y

             General approach:
                Vec3f a(0, params0.y, 0);
                Vec3f b(0, -params0.y, 0);

                Vec3f ba = b - a, pa = localPosition - a;
                float h = clamp(pa.dot(ba) / (ba.dot(ba)), 0.0, 1.0);
                Vec3f q = pa - ba * h;
                float d = q.norm();
            */

            const float cinlinderHeight = params0.y * 2;
            Vec3f pa = localPosition;
            pa.y() -= params0.y;
            const float h = clamp(-pa.y() / cinlinderHeight, 0.0, 1.0);
            Vec3f &q = pa;
            q.y() += cinlinderHeight * h;
            const float d = q.norm();

            Vec3f gradient = q / fmaxf(d, CR_EPS);

            sdfGradientDistance = gradient.data;
            sdfGradientDistance.w = d - params0.x;
            break;
        }

        case GeometryType::eArc:
        {
            // An arc is located on the X-Z plane, centered at origin with unit radius
            // The arc starts at params0.x, and ends at params0.y in radian
            // Must be within (-pi, pi)
            // Points that do not project on the arc are discarded.
            float angleLower = params0.x;
            float angleUpper = params0.y;

            float pointAngle = atan2f(localPosition.z(), localPosition.x());

            if (pointAngle < angleLower || pointAngle > angleUpper)
            {
                sdfGradientDistance.x = 0;
                sdfGradientDistance.y = 1;
                sdfGradientDistance.z = 0;
                sdfGradientDistance.w = nanf("0");
                break;
            }

            Vec3f projectedPoint(cosf(pointAngle), 0, sinf(pointAngle));
            Vec3f projectionToPoint = localPosition - projectedPoint;
            float distance = projectionToPoint.norm();
            sdfGradientDistance = (projectionToPoint / fmaxf(distance, CR_EPS)).data;
            sdfGradientDistance.w = distance;
            if (tangentDirection)
            {
                tangentDirection->x() = -projectedPoint.z();
                tangentDirection->y() = 0;
                tangentDirection->z() = projectedPoint.x();
            }

            break;
        }

        case GeometryType::eTriangleMesh:
        {
            // For precomputed SDF grid
            const unsigned int sdfDataOffset = CR_FLOAT_AS_INT(params0.x);
            const unsigned int sdfDataId = CR_FLOAT_AS_INT(params0.y);
            float4 lowerBoundCellSize = sdfLowerBoundCellSize[sdfDataId];
            Vec3i sdfDim = sdfDimension[sdfDataId];

            Vec3f lowerBound(lowerBoundCellSize);

            Vec3f scaledPos = Vec3f(localPosition - lowerBound) / lowerBoundCellSize.w;
            Vec3i mappedQueryPoint = (scaledPos + crmpm::Vec3f(0.5f)).cast<int>(); // Add 0.5 then cast down results in the closest node.

            // If outside bounds, set distance to max float.
            if (mappedQueryPoint.x() < 0 || mappedQueryPoint.x() >= sdfDim.x() ||
                mappedQueryPoint.y() < 0 || mappedQueryPoint.y() >= sdfDim.y() ||
                mappedQueryPoint.z() < 0 || mappedQueryPoint.z() >= sdfDim.z())
            {
                sdfGradientDistance.x = 0;
                sdfGradientDistance.y = 1;
                sdfGradientDistance.z = 0;
                sdfGradientDistance.w = CR_MAX_F32;
                break;
            }

            int sdfQueryIndex = getGridIndex(sdfDim, sdfDataOffset, mappedQueryPoint);

            // Directly use the gradient of nearest node
            sdfGradientDistance.x = sdfGridData[sdfQueryIndex].x;
            sdfGradientDistance.y = sdfGridData[sdfQueryIndex].y;
            sdfGradientDistance.z = sdfGridData[sdfQueryIndex].z;

            // Interpolate the distance
            Vec3i baseCoord = scaledPos.cast<int>();
            Vec3f fracPos = scaledPos - baseCoord.cast<float>();

            // Check again in case baseCoord is outsize or at the upperbound
            if (baseCoord.x() < 0 || baseCoord.x() >= sdfDim.x() - 1 ||
                baseCoord.y() < 0 || baseCoord.y() >= sdfDim.y() - 1 ||
                baseCoord.z() < 0 || baseCoord.z() >= sdfDim.z() - 1)
            {
                sdfGradientDistance.w = sdfGridData[sdfQueryIndex].w;
                break;
            }

            float gridValues[8];
            #ifdef __CUDA_ARCH__
            #pragma unroll 8
            #endif
            for (int dx = 0; dx <= 1; ++dx)
            {
                for (int dy = 0; dy <= 1; ++dy)
                {
                    for (int dz = 0; dz <= 1; ++dz)
                    {
                        int idx = (dx << 2) | (dy << 1) | dz;
                        gridValues[idx] = sdfGridData[getGridIndex(sdfDim, sdfDataOffset, baseCoord.x() + dx, baseCoord.y() + dy, baseCoord.z() + dz)].w;
                    }
                }
            }

            // Trilinear interpolation
            float fx = fracPos.x(), fy = fracPos.y(), fz = fracPos.z();
            float oneMinusFx = 1.0f - fx, oneMinusFy = 1.0f - fy, oneMinusFz = 1.0f - fz;

            float weights[8] = {
                oneMinusFx * oneMinusFy * oneMinusFz,
                fx * oneMinusFy * oneMinusFz,
                oneMinusFx * fy * oneMinusFz,
                fx * fy * oneMinusFz,
                oneMinusFx * oneMinusFy * fz,
                fx * oneMinusFy * fz,
                oneMinusFx * fy * fz,
                fx * fy * fz,
            };

            float interpolatedValue = 0;
            #ifdef __CUDA_ARCH__
            #pragma unroll 8
            #endif
            for (int i = 0; i < 8; ++i)
            {
                interpolatedValue += gridValues[i] * weights[i];
            }

            sdfGradientDistance.w = interpolatedValue;

            break;
        }

        // For slicer geometries, use unsigned distance and flip the normal if inside.
        // This requires a slightly fattened geometry to allow collision response

        case GeometryType::eQuadSlicer:
        {
            // Half size
            float halfX = params0.x;
            float halfZ = params0.y;

            if (localPosition.x() < -halfX || localPosition.x() > halfX ||
                localPosition.z() < -halfZ || localPosition.z() > halfZ)
            {
                sdfGradientDistance.x = 0.0f;
                sdfGradientDistance.y = 0.0f;
                sdfGradientDistance.z = 0.0f;
                sdfGradientDistance.w = CR_MAX_F32;
            }
            else
            {
                sdfGradientDistance.x = 0.0f;
                sdfGradientDistance.y = copysignf(1.0f, localPosition.y());
                sdfGradientDistance.z = 0.0f;
                sdfGradientDistance.w = fabsf(localPosition.y());
            }
            break;
        }

        case GeometryType::eTriangleMeshSlicer:
        {
            sdfGradientDistance.x = 0;
            sdfGradientDistance.y = 1;
            sdfGradientDistance.z = 0;
            sdfGradientDistance.w = nanf("0");

            // Same as triangle mesh, but the distance will take its abs value.
            const unsigned int sdfDataOffset = CR_FLOAT_AS_INT(params0.x) + 1;
            const unsigned int sdfDataId = CR_FLOAT_AS_INT(params0.y);
            float4 lowerBoundCellSize = sdfLowerBoundCellSize[sdfDataId];
            Vec3i sdfDim = sdfDimension[sdfDataId];

            Vec3f lowerBound(lowerBoundCellSize);

            Vec3f scaledPos = Vec3f(localPosition - lowerBound) / lowerBoundCellSize.w;
            Vec3i mappedQueryPoint = (scaledPos + crmpm::Vec3f(0.5f)).cast<int>(); // Add 0.5 then cast down results in the closest node.

            // If outside or at the bounds
            // this shrinks that actual size, but make sense as the boundary gradients are not good anyway
            if (mappedQueryPoint.x() < 1 || mappedQueryPoint.x() >= sdfDim.x() - 1 ||
                mappedQueryPoint.y() < 1 || mappedQueryPoint.y() >= sdfDim.y() - 1 ||
                mappedQueryPoint.z() < 1 || mappedQueryPoint.z() >= sdfDim.z() - 1)
            {
                break;
            }

            int sdfQueryIndex = getGridIndex(sdfDim, sdfDataOffset, mappedQueryPoint);
            float queriedGradientSideDistance = sdfGridData[sdfQueryIndex].w;
            if (isnan(queriedGradientSideDistance))
            {
                break;
            }

            // Directly use the gradient of nearest node
            sdfGradientDistance.x = sdfGridData[sdfQueryIndex].x;
            sdfGradientDistance.y = sdfGridData[sdfQueryIndex].y;
            sdfGradientDistance.z = sdfGridData[sdfQueryIndex].z;

            // Interpolate the distance
            Vec3i baseCoord = scaledPos.cast<int>();
            Vec3f fracPos = scaledPos - baseCoord.cast<float>();

            float gridValues[8];
            #ifdef __CUDA_ARCH__
            #pragma unroll 8
            #endif
            for (int dx = 0; dx <= 1; ++dx)
            {
                for (int dy = 0; dy <= 1; ++dy)
                {
                    for (int dz = 0; dz <= 1; ++dz)
                    {
                        int idx = (dx << 2) | (dy << 1) | dz;
                        gridValues[idx] = sdfGridData[getGridIndex(sdfDim, sdfDataOffset, baseCoord.x() + dx, baseCoord.y() + dy, baseCoord.z() + dz)].w;
                    }
                }
            }

            // Trilinear interpolation
            float fx = fracPos.x(), fy = fracPos.y(), fz = fracPos.z();
            float oneMinusFx = 1.0f - fx, oneMinusFy = 1.0f - fy, oneMinusFz = 1.0f - fz;

            float weights[8] = {
                oneMinusFx * oneMinusFy * oneMinusFz,
                fx * oneMinusFy * oneMinusFz,
                oneMinusFx * fy * oneMinusFz,
                fx * fy * oneMinusFz,
                oneMinusFx * oneMinusFy * fz,
                fx * oneMinusFy * fz,
                oneMinusFx * fy * fz,
                fx * fy * fz,
            };

            float interpolatedValue = 0;
            #ifdef __CUDA_ARCH__
            #pragma unroll 8
            #endif
            for (int i = 0; i < 8; ++i)
            {
                interpolatedValue += gridValues[i] * weights[i];
            }

            if (isnan(interpolatedValue))
            {
                break;
            }

            // Assign the final abs distance value
            sdfGradientDistance.w = fabsf(interpolatedValue);

            // Check if after interpolation, the side changes compared to the gradient query point
            if (queriedGradientSideDistance * interpolatedValue < 0)
            {
                sdfGradientDistance.x = -sdfGradientDistance.x;
                sdfGradientDistance.y = -sdfGradientDistance.y;
                sdfGradientDistance.z = -sdfGradientDistance.z;
            }

            // Check if the point is in the spine area
            float4 spinePlane = sdfGridData[sdfDataOffset - 1];
            if (isnan(spinePlane.x))
            {
                break;
            }
            Vec3f spineNormal = Vec3f(spinePlane);
            float length = spineNormal.norm();
            float side = localPosition.dot(spineNormal / length) + spinePlane.w;
            if (side > 0)
            {
                // scale the distance according the normal for spine
                sdfGradientDistance.w -= length - 1;
                inSpine = true;
            }

            break;
        }

        case GeometryType::eConnectedLineSegments:
        {
            sdfGradientDistance.x = 0;
            sdfGradientDistance.y = 1;
            sdfGradientDistance.z = 0;
            sdfGradientDistance.w = nanf("0");

            const unsigned int sdfDataOffset = CR_FLOAT_AS_INT(params0.x);
            const unsigned int sdfDataId = CR_FLOAT_AS_INT(params0.y);
            const unsigned int sdfDataSize = CR_FLOAT_AS_INT(params0.z);

            Bounds3 bounds;
            bounds.minimum = sdfLowerBoundCellSize[sdfDataId];
            bounds.maximum = sdfGridData[sdfDataOffset];

            if (!bounds.contains(localPosition))
            {
                break;
            }

            // Loop over points and cast the query point to each line segment
            // I don't know if this can be made faster
            int closestSegmentPointIdx = 0;
            Vec3f closestPoint;
            float closestDistance = CR_MAX_F32;
            Vec3f point0;
            Vec3f point1;
            for (int p = sdfDataOffset + 1; p < sdfDataOffset + sdfDataSize - 1; ++p)
            {
                point0 = sdfGridData[p];
                point1 = sdfGridData[p + 1];
                Vec3f lineVec = point1 - point0;

                // Cast to point on line
                Vec3f pointVec = localPosition - point0;

                float t = pointVec.dot(lineVec) / lineVec.squaredNorm();
                t = fmaxf(0.0f, fminf(1.0f, t)); // Clamp t to [0,1]

                Vec3f projectedPoint = point0 + lineVec * t;
                float distance = (localPosition - projectedPoint).norm();

                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestPoint = projectedPoint;
                    closestSegmentPointIdx = p;
                }
            }

            // Disgard points that cast outside the line
            // point1 is already the last point
            point0 = sdfGridData[sdfDataOffset + 1];
            if ((closestPoint - point0).squaredNorm() < CR_EPS || (closestPoint - point1).squaredNorm() < CR_EPS)
            {
                break;
            }

            Vec3f gradient = localPosition - closestPoint;
            gradient.normalize();
            if (tangentDirection)
            {
                *tangentDirection = sdfGridData[closestSegmentPointIdx] - sdfGridData[closestSegmentPointIdx + 1];
                tangentDirection->normalize();
            }

            sdfGradientDistance = gradient.data;
            sdfGradientDistance.w = closestDistance;
            break;
        }

        default:
            break;
        }

        return inSpine;
    }

    /**
     * @brief Compute geometry Modified SDF, same as getGeometrySdf but only for slicer geometries
     * and check whether the point projects on the geometry surface.
     *
     * @return bool - Whether the point projects onto the geometry
     */
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE bool getGeometryModSdf(
        const Vec3f localPosition,
        const GeometryType type,
        const float4 params0,
        const Vec3i *CR_RESTRICT sdfDimension,
        const float4 *CR_RESTRICT sdfLowerBoundCellSize,
        const float4 *CR_RESTRICT sdfGridData,
        float4 &sdfGradientDistance)
    {
        bool projectionOnGeom = false;
        switch (type)
        {
        case GeometryType::eQuadSlicer:
        {
            // Half size
            const float halfX = params0.x;
            const float halfZ = params0.y;

            if (localPosition.x() < -halfX || localPosition.x() > halfX ||
                localPosition.z() < -halfZ || localPosition.z() > halfZ)
            {
                return false;
            }

            projectionOnGeom = true;
            sdfGradientDistance.x = 0.0f;
            sdfGradientDistance.y = copysignf(1.0f, localPosition.y());
            sdfGradientDistance.z = 0.0f;
            sdfGradientDistance.w = localPosition.y();
            break;
        }

        case GeometryType::eTriangleMeshSlicer:
        {
            // Same as triangle mesh, but the distance will be taken abs value.
            projectionOnGeom = false;
            sdfGradientDistance.x = 0;
            sdfGradientDistance.y = 1;
            sdfGradientDistance.z = 0;
            sdfGradientDistance.w = nanf("0");

            unsigned int sdfDataOffset = CR_FLOAT_AS_INT(params0.x);

            // Check if the point is in the spine area
            float4 spinePlane = sdfGridData[sdfDataOffset];
            if (isnan(spinePlane.x))
            {
                break;
            }
            float side = localPosition.dot(Vec3f(spinePlane)) + spinePlane.w;
            if (side > 0)
            {
                break;
            }

            sdfDataOffset++; // Actual start of the SDF data

            const unsigned int sdfDataId = CR_FLOAT_AS_INT(params0.y);
            float4 lowerBoundCellSize = sdfLowerBoundCellSize[sdfDataId];
            Vec3i sdfDim = sdfDimension[sdfDataId];

            Vec3f lowerBound(lowerBoundCellSize);

            Vec3f scaledPos = Vec3f(localPosition - lowerBound) / lowerBoundCellSize.w;
            Vec3i mappedQueryPoint = (scaledPos + crmpm::Vec3f(0.5f)).cast<int>(); // Add 0.5 then cast down results in the closest node.

            // If outside or at the bounds
            // this shrinks that actual size, but make sense as the boundary gradients are not good anyway
            if (mappedQueryPoint.x() < 1 || mappedQueryPoint.x() >= sdfDim.x() - 1 ||
                mappedQueryPoint.y() < 1 || mappedQueryPoint.y() >= sdfDim.y() - 1 ||
                mappedQueryPoint.z() < 1 || mappedQueryPoint.z() >= sdfDim.z() - 1)
            {
                break;
            }

            int sdfQueryIndex = getGridIndex(sdfDim, sdfDataOffset, mappedQueryPoint);
            float queriedGradientSideDistance = sdfGridData[sdfQueryIndex].w;
            if (isnan(queriedGradientSideDistance))
            {
                break;
            }

            // Directly use the gradient of nearest node
            sdfGradientDistance.x = sdfGridData[sdfQueryIndex].x;
            sdfGradientDistance.y = sdfGridData[sdfQueryIndex].y;
            sdfGradientDistance.z = sdfGridData[sdfQueryIndex].z;

            // Interpolate the distance
            Vec3i baseCoord = scaledPos.cast<int>();
            Vec3f fracPos = scaledPos - baseCoord.cast<float>();

            float gridValues[8];
            #ifdef __CUDA_ARCH__
            #pragma unroll 8
            #endif
            for (int dx = 0; dx <= 1; ++dx)
            {
                for (int dy = 0; dy <= 1; ++dy)
                {
                    for (int dz = 0; dz <= 1; ++dz)
                    {
                        int idx = (dx << 2) | (dy << 1) | dz;
                        gridValues[idx] = sdfGridData[getGridIndex(sdfDim, sdfDataOffset, baseCoord.x() + dx, baseCoord.y() + dy, baseCoord.z() + dz)].w;
                    }
                }
            }

            // Trilinear interpolation
            float fx = fracPos.x(), fy = fracPos.y(), fz = fracPos.z();
            float oneMinusFx = 1.0f - fx, oneMinusFy = 1.0f - fy, oneMinusFz = 1.0f - fz;

            float weights[8] = {
                oneMinusFx * oneMinusFy * oneMinusFz,
                fx * oneMinusFy * oneMinusFz,
                oneMinusFx * fy * oneMinusFz,
                fx * fy * oneMinusFz,
                oneMinusFx * oneMinusFy * fz,
                fx * oneMinusFy * fz,
                oneMinusFx * fy * fz,
                fx * fy * fz,
            };

            float interpolatedValue = 0;
            #ifdef __CUDA_ARCH__
            #pragma unroll 8
            #endif
            for (int i = 0; i < 8; ++i)
            {
                interpolatedValue += gridValues[i] * weights[i];
            }

            if (isnan(interpolatedValue))
            {
                break;
            }

            // At this point, we can finally say that the point projects on the mesh.
            projectionOnGeom = true;

            // Assign the final distance value
            sdfGradientDistance.w = interpolatedValue;

            // Check if after interpolation, the side changes compared to the gradient query point
            if (queriedGradientSideDistance * interpolatedValue < 0)
            {
                sdfGradientDistance.x = -sdfGradientDistance.x;
                sdfGradientDistance.y = -sdfGradientDistance.y;
                sdfGradientDistance.z = -sdfGradientDistance.z;
            }

            break;
        }

        default:
            break;
        }

        return projectionOnGeom;
    }
} // namespace crmpm

#endif // !CR_SDF_H